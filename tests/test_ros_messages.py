import pytest


def test_actual_ros_serialization():
    pytest.importorskip('rclpy',reason='Requires ROS 2 Humble installation; not available on this Mac')
    from rclpy.serialization import serialize_message, deserialize_message
    from mavros_msgs.msg import OverrideRCIn, RCOut
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from blueboat_ppo_controller.mavros_interface import rc_override
    msg=OverrideRCIn(channels=rc_override([1,3],[1500,1500]))
    restored=deserialize_message(serialize_message(msg),OverrideRCIn)
    assert list(restored.channels)==list(msg.channels)
    command=Twist(); command.linear.x=.4; command.angular.z=.2
    restored=deserialize_message(serialize_message(command),Twist)
    assert restored.linear.x==.4 and restored.angular.z==.2
    raw=Imu(); raw.header.frame_id='base_link'; raw.orientation_covariance[0]=-1.
    raw.linear_acceleration.z=9.80665
    restored=deserialize_message(serialize_message(raw),Imu)
    assert restored.orientation_covariance[0]==-1. and restored.linear_acceleration.z==9.80665
    odom=Odometry(); odom.header.frame_id='map'; odom.child_frame_id='base_link'
    restored=deserialize_message(serialize_message(odom),Odometry)
    assert restored.child_frame_id=='base_link'
    output=RCOut(); output.channels=[1500,0,1510]
    restored=deserialize_message(serialize_message(output),RCOut)
    assert list(restored.channels)==[1500,0,1510]
