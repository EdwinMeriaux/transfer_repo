"""Relay MAVROS telemetry with scheduled dropouts, for SITL and unarmed gap tests.

Every message is either forwarded unmodified (timestamps untouched) or
dropped. Point the calibration or PPO node at the relayed topics
(``--make-config``). This node never publishes motor commands.

    gap_injector --make-config IN.yaml OUT.yaml      # copy a config, retargeted
    gap_injector --ros-args -p gap_lengths_s:="[0.25, 1.2]" -p gap_every_s:=8.0
"""
import argparse
import sys
from pathlib import Path

import yaml

PREFIX = '/blueboat/gap_test'
# Telemetry topics each node reads, and the relayed name used for them.
NODE_TOPICS = {
    'blueboat_thrust_calibration': {'raw_imu_topic': 'imu/data_raw', 'attitude_topic': 'imu/data',
                                    'velocity_topic': 'local_position/odom', 'output_topic': 'rc/out'},
    'blueboat_ppo_controller': {'imu_topic': 'imu/data', 'odometry_topic': 'local_position/odom'},
}


def retarget_config(source, destination, prefix=PREFIX):
    """Copy a calibration or controller YAML with its telemetry topics relayed."""
    config = yaml.safe_load(Path(source).read_text())
    unknown = set(config) - set(NODE_TOPICS)
    if unknown:
        raise ValueError('Not a calibration or controller config: ' + ', '.join(sorted(unknown)))
    for node, settings in ((n, c['ros__parameters']) for n, c in config.items()):
        for key, relayed in NODE_TOPICS[node].items():
            settings[key] = prefix + '/' + relayed
        if settings.get('log_directory'):
            settings['log_directory'] = str(Path(settings['log_directory']) / 'gap-test')
    Path(destination).write_text(yaml.safe_dump(config, sort_keys=False))
    return config


def gap_active(elapsed_s, start_after_s, every_s, lengths_s):
    """Index of the gap in progress at elapsed_s, or None."""
    if not lengths_s or elapsed_s < start_after_s:
        return None
    index, offset = divmod(elapsed_s - start_after_s, every_s)
    length = lengths_s[int(index) % len(lengths_s)]
    return int(index) if offset < length else None


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    if argv[:1] == ['--make-config']:
        parser = argparse.ArgumentParser(prog='gap_injector --make-config')
        parser.add_argument('flag')
        parser.add_argument('source', type=Path)
        parser.add_argument('destination', type=Path)
        parser.add_argument('--prefix', default=PREFIX)
        parsed = parser.parse_args(argv)
        if parsed.destination.exists():
            parser.error('destination exists; choose a new file')
        retarget_config(parsed.source, parsed.destination, parsed.prefix)
        print(f'Wrote {parsed.destination}: telemetry topics relayed through {parsed.prefix}/...')
        return 0
    return run_relay(args)


def run_relay(args=None):
    import time
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import RCOut

    class GapInjector(Node):
        def __init__(self):
            super().__init__('blueboat_gap_injector')
            defaults = dict(prefix=PREFIX, start_after_s=10., gap_every_s=8., gap_lengths_s=[.25, 1.2],
                            drop_output=False)
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            s = {name: self.get_parameter(name).value for name in defaults}
            if s['gap_every_s'] <= max(s['gap_lengths_s'] or [0.]) or any(v <= 0 for v in s['gap_lengths_s']):
                raise ValueError('Each gap must be positive and shorter than gap_every_s')
            self.s = s
            self.started = time.monotonic()
            self.current = None
            relays = [('/mavros/imu/data_raw', 'imu/data_raw', Imu, False),
                      ('/mavros/imu/data', 'imu/data', Imu, False),
                      ('/mavros/local_position/odom', 'local_position/odom', Odometry, False),
                      ('/mavros/rc/out', 'rc/out', RCOut, True)]
            self.dropped = {}
            for source, name, kind, is_output in relays:
                publisher = self.create_publisher(kind, s['prefix'] + '/' + name, qos_profile_sensor_data)
                self.dropped[name] = 0
                self.create_subscription(kind, source, self.relay(publisher, name, is_output), qos_profile_sensor_data)
            self.get_logger().info(f'Relaying to {s["prefix"]}/...; first gap after {s["start_after_s"]} s, '
                                   f'every {s["gap_every_s"]} s, lengths {s["gap_lengths_s"]} s')

        def relay(self, publisher, name, is_output):
            def callback(message):
                gap = gap_active(time.monotonic() - self.started, self.s['start_after_s'],
                                 self.s['gap_every_s'], self.s['gap_lengths_s'])
                if gap != self.current:
                    if gap is not None:
                        length = self.s['gap_lengths_s'][gap % len(self.s['gap_lengths_s'])]
                        self.get_logger().info(f'gap {gap}: dropping telemetry for {length} s')
                    else:
                        self.get_logger().info(f'gap {self.current} over; dropped so far {self.dropped}')
                    self.current = gap
                if gap is not None and (not is_output or self.s['drop_output']):
                    self.dropped[name] += 1
                    return
                publisher.publish(message)
            return callback

    rclpy.init(args=args)
    node = GapInjector()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
