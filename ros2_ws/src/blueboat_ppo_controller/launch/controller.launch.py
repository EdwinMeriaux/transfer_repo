from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default = PathJoinSubstitution([FindPackageShare('blueboat_ppo_controller'), 'config', 'controller.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('mode', default_value='shadow'),
        DeclareLaunchArgument('enable_actuators', default_value='false'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('deployment_target', default_value='sitl'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='blueboat_ppo_controller', executable='ppo_node', name='blueboat_ppo_controller', output='screen',
            parameters=[LaunchConfiguration('config'), {
                'mode': ParameterValue(LaunchConfiguration('mode'), value_type=str),
                'enable_actuators': ParameterValue(LaunchConfiguration('enable_actuators'), value_type=bool),
                'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'), value_type=str),
                'deployment_target': ParameterValue(LaunchConfiguration('deployment_target'), value_type=str),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])])
