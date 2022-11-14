from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="storm32_gimbal",
            namespace="gimbal",
            executable="gimbal",
            name="controller",
            output="screen",
            respawn=True,
            parameters=[
                {"frame_id": "gimbal_ref"},
                {"port": "/dev/tty_gimbal"}
            ]
        )
    ])
