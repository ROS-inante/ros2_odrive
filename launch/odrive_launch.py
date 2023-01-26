from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_odrive",
            executable="odrive",
            name="odrive_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"serial_number": "0x11111111"}
            ]
        )
    ])
