import pytest


def test_actual_ros_serialization():
    pytest.importorskip('rclpy',reason='Requires ROS 2 Humble installation; not available on this Mac')
    from rclpy.serialization import serialize_message, deserialize_message
    from mavros_msgs.msg import OverrideRCIn
    from geometry_msgs.msg import Twist
    from blueboat_ppo_controller.mavros_interface import rc_override
    msg=OverrideRCIn(channels=rc_override([1,3],[1500,1500]))
    restored=deserialize_message(serialize_message(msg),OverrideRCIn)
    assert list(restored.channels)==list(msg.channels)
    command=Twist(); command.linear.x=.4; command.angular.z=.2
    restored=deserialize_message(serialize_message(command),Twist)
    assert restored.linear.x==.4 and restored.angular.z==.2
