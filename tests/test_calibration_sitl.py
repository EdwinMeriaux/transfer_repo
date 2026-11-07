"""Live SITL acceptance. External operator arms/selects MANUAL/enables simulated RC.

This suite deliberately transmits PWM when explicitly opted in. It never arms,
sets flight modes, changes FC parameters, or sets hardware verification flags.
"""
import csv
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace as S

import pytest

pytestmark = [pytest.mark.sitl, pytest.mark.skipif(os.environ.get('BLUEBOAT_RUN_POWERED_SITL') != '1',
    reason='Live powered ArduPilot SITL requires explicit opt-in and an isolated, operator-prepared ROS graph')]


@pytest.fixture
def live_sitl(tmp_path):
    rclpy = pytest.importorskip('rclpy', reason='ROS 2 Humble is not installed')
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, String
    from mavros_msgs.msg import State, RCOut, OverrideRCIn
    import yaml
    from blueboat_ppo_controller.motor_interface import MotorInterface
    path = Path(os.environ['BLUEBOAT_SITL_INTERFACE']).resolve()
    motor = MotorInterface(path, 'sitl')
    mass = float(os.environ['BLUEBOAT_SITL_MASS_KG'])
    assert mass > 0
    settings = dict(state_topic='/mavros/state', output_topic='/mavros/rc/out',
        rc_override_topic='/mavros/rc/override', enable_topic='/blueboat/calibration/enable',
        diagnostics_topic='/blueboat/calibration/status')
    config = os.environ.get('BLUEBOAT_SITL_CONFIG')
    if config:
        settings.update(yaml.safe_load(Path(config).read_text())['blueboat_thrust_calibration']['ros__parameters'])
    rclpy.init()
    observer = rclpy.create_node('thrust_calibration_sitl_observer')
    state = S(vehicle=None, output=None, overrides=[], statuses=[], enable=False, process=None)
    subscriptions = [
        observer.create_subscription(State, settings['state_topic'], lambda m:setattr(state,'vehicle',m), 10),
        observer.create_subscription(RCOut, settings['output_topic'], lambda m:setattr(state,'output',m), qos_profile_sensor_data),
        observer.create_subscription(OverrideRCIn, settings['rc_override_topic'], lambda m:state.overrides.append((time.monotonic(),list(m.channels))), 10),
        observer.create_subscription(String, settings['diagnostics_topic'], lambda m:state.statuses.append(json.loads(m.data)), 10)]
    enable = observer.create_publisher(Bool, settings['enable_topic'], 1)
    timer = observer.create_timer(.1, lambda:enable.publish(Bool(data=state.enable)))

    def wait_for(predicate, timeout, description):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=.02)
            if predicate():
                return
            if state.process is not None and state.process.poll() is not None:
                pytest.fail('Calibration subprocess exited; inspect '+str(tmp_path/'node.log'))
        pytest.fail('SITL timeout: '+description+'; logs: '+str(tmp_path))

    try:
        wait_for(lambda:state.vehicle is not None and state.vehicle.connected and state.vehicle.armed
                 and state.vehicle.mode == 'MANUAL', 15., 'operator must arm SITL and select MANUAL externally')
        command = [sys.executable, '-c', 'from blueboat_ppo_controller.calibration_node import main; main()', '--ros-args']
        if config:
            command += ['--params-file', config]
        params = dict(mode='powered', enable_powered=True, deployment_target='sitl', interface_file=str(path),
            mass_kg=mass, log_directory=str(tmp_path), repeats=1, delta_pwm=[40],
            sides=['left','right','symmetric'], baseline_s=2., pulse_s=.4, recovery_s=4., require_output_feedback=True)
        for key,value in params.items():
            command += ['-p', key+':='+json.dumps(value)]
        environment = dict(os.environ)
        package = str(Path(__file__).resolve().parents[1]/'ros2_ws/src/blueboat_ppo_controller')
        environment['PYTHONPATH'] = package+os.pathsep+environment.get('PYTHONPATH','')
        with (tmp_path/'node.log').open('w') as logfile:
            state.process = subprocess.Popen(command, env=environment, stdout=logfile, stderr=subprocess.STDOUT)
        wait_for(lambda:any(s['status']=='enable_released' for s in state.statuses), 15., 'preflight telemetry + MANUAL gate')
        assert not state.overrides, 'No overrides may be sent before the software enable heartbeat'
        state.enable = True
        state.wait_for, state.motor, state.directory = wait_for, motor, tmp_path
        yield state
    finally:
        state.enable = False
        enable.publish(Bool(data=False))
        if state.process is not None:
            if state.process.poll() is None:
                state.process.send_signal(signal.SIGCONT)
                state.process.send_signal(signal.SIGINT)
                try:
                    state.process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    state.process.kill()
                    state.process.wait(timeout=5.)
        timer.cancel()
        observer.destroy_node()
        rclpy.shutdown()


def nonneutral(state):
    return any(any(pwm[c-1] not in (0,65535,n) for c,n in zip(state.motor.channels,state.motor.neutral()))
               for _,pwm in state.overrides)


def outputs_neutral(state):
    return state.output is not None and all(len(state.output.channels) >= state.motor.config[side]['servo_output']
        and abs(state.output.channels[state.motor.config[side]['servo_output']-1]-state.motor.config[side]['servo_neutral_pwm']) <= 15
        for side in ('left','right'))


def test_live_sitl_individual_and_symmetric_outputs(live_sitl):
    state=live_sitl
    state.wait_for(lambda:any(s['status']=='complete' for s in state.statuses), 45., 'finite pulse plan completes without a fault')
    csv_path=next(state.directory.glob('*.csv'))
    with csv_path.open() as stream:
        rows=list(csv.DictReader(stream))
    for side in ('left','right','symmetric'):
        pulse=[r for r in rows if r['phase']=='pulse' and r['side']==side]
        assert pulse, side
        assert all(r['valid']=='1' for r in pulse)
        for motor_side in ('left','right'):
            neutral=state.motor.config[motor_side]['servo_neutral_pwm']
            values=[float(r['output_'+motor_side+'_pwm']) for r in pulse if r['output_'+motor_side+'_pwm']]
            assert values
            if side in (motor_side,'symmetric'):
                assert max(abs(v-neutral) for v in values)>3, 'Actual output did not respond'
            else:
                assert max(abs(v-neutral) for v in values)<=15, 'Idle motor was coupled'
    assert all(state.overrides[-1][1][c-1]==0 for c in state.motor.channels)
    state.wait_for(lambda:outputs_neutral(state), 2., 'receiver/FC neutral after release')


def test_live_sitl_enable_loss_latches_and_releases(live_sitl):
    state=live_sitl
    state.wait_for(lambda:nonneutral(state), 20., 'first nonneutral PWM pulse')
    state.enable=False
    state.wait_for(lambda:state.overrides and all(state.overrides[-1][1][c-1]==0 for c in state.motor.channels), 1., 'software takeover release')
    released_count=len(state.overrides)
    state.enable=True
    state.wait_for(lambda:any(s['status']=='enable_released' and s['trial']>=0 for s in state.statuses), 2., 'latched stop diagnostic')
    assert len(state.overrides)==released_count
    state.wait_for(lambda:outputs_neutral(state), 2., 'actual neutral after takeover')


def test_live_sitl_process_stall_exercises_fc_override_timeout(live_sitl):
    state=live_sitl
    state.wait_for(lambda:nonneutral(state), 20., 'first nonneutral PWM pulse')
    state.wait_for(lambda:not outputs_neutral(state), 1., 'actual servo response before process stall')
    state.process.send_signal(signal.SIGSTOP)
    state.wait_for(lambda:outputs_neutral(state), 1.5, 'FC RC_OVERRIDE_TIME restores tested neutral receiver inputs')
    state.process.send_signal(signal.SIGCONT)
