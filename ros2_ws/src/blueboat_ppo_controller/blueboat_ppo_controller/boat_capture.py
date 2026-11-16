import json
import time


def capture(directory, profile):
    import rclpy
    from rclpy.node import Node
    from mavros_msgs.srv import ParamPull, VehicleInfoGet
    from rcl_interfaces.srv import GetParameters
    import yaml
    from .boat_profile import CHECKS, write_json

    settings = yaml.safe_load((directory/'thrust_calibration.yaml').read_text())['blueboat_thrust_calibration']['ros__parameters']
    rclpy.init()
    node = Node('blueboat_read_only_configuration')
    try:
        requests = [
            (VehicleInfoGet, settings.get('vehicle_info_service','/mavros/vehicle_info_get'), VehicleInfoGet.Request(sysid=0,compid=0,get_all=False)),
            (ParamPull, settings.get('param_pull_service','/mavros/param/pull'), ParamPull.Request(force_pull=True)),
            (GetParameters, settings.get('param_get_service','/mavros/param/get_parameters'), GetParameters.Request(names=list(profile['expected_parameters'])))
        ]
        responses = []
        for kind, name, request in requests:
            client = node.create_client(kind, name)
            if not client.wait_for_service(timeout_sec=10.):
                raise ValueError('Service unavailable: ' + name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=60.)
            if not future.done() or future.exception() is not None:
                raise ValueError('Read-only service failed/timed out: ' + name)
            responses.append(future.result())
        vehicles, pull, parameters = responses
        if not vehicles.success or len(vehicles.vehicles) != 1 or not pull.success:
            raise ValueError('Vehicle identity / parameter refresh failed')
        info = vehicles.vehicles[0]
        if not info.available_info & 2 or info.autopilot != 3 or info.type not in (10,11):
            raise ValueError('Expected one ArduPilot rover/boat with known firmware identity')
        if info.sysid != profile['sysid']:
            raise ValueError('Unexpected system ID; do not adopt another vehicle')
        if not 1 <= info.compid <= 255 or not info.flight_sw_version:
            raise ValueError('Missing component or firmware identity')
        actual = {}
        if len(parameters.values) != len(profile['expected_parameters']):
            raise ValueError('Incomplete parameter response')
        for name, value in zip(profile['expected_parameters'], parameters.values):
            if value.type not in (2,3):
                raise ValueError('Missing/non-numeric live parameter: ' + name)
            actual[name] = value.integer_value if value.type == 2 else value.double_value
        differences = {key:dict(expected=value, actual=actual[key]) for key,value in profile['expected_parameters'].items()
                       if abs(actual[key]-value) > 1e-5}
        identity = {key:getattr(info,key) for key in ('sysid','compid','flight_sw_version')}
        snapshot = dict(identity=identity,
                        identity_basis='system_component_firmware_and_parameters',
                        actual_parameters=actual, parameter_differences=differences,
                        note='Read-only MAVROS capture. No parameters written, motors commanded, or tests marked passed.')
        filename = directory/f'live-capture-{time.time_ns()}.json'
        write_json(filename, snapshot)
        version = info.flight_sw_version
        profile.pop('uid', None)
        profile.update(identity, firmware_version=f'ArduRover {version >> 24}.{(version >> 16) & 255}.{(version >> 8) & 255} (type {version & 255}, captured)',
                       verified=False, **{key:False for key in CHECKS})
        write_json(directory/'motor_interface.json', profile)
        print(json.dumps(dict(snapshot=str(filename), identity=identity, parameter_differences=differences,
                              next_step='Review identity and differences; refresh from a new export if settings intentionally changed. Test evidence is still required.'),indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
