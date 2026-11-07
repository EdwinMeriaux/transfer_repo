from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default = PathJoinSubstitution([FindPackageShare('blueboat_ppo_controller'), 'config', 'thrust_calibration.yaml'])
    types = dict(mode=('telemetry', str), enable_powered=('false', bool), deployment_target=('sitl', str),
                 interface_file=('', str), use_sim_time=('false', bool))
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default)] + [
        DeclareLaunchArgument(name, default_value=value) for name,(value, _) in types.items()] + [
        Node(package='blueboat_ppo_controller', executable='thrust_calibration', name='blueboat_thrust_calibration', output='screen',
             parameters=[LaunchConfiguration('config'), {name:ParameterValue(LaunchConfiguration(name), value_type=kind)
                                                       for name,(_, kind) in types.items()}])])
