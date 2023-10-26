from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
    Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="service_node.py",    # Enter the name of your executable
    ),
    Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="controller.py",    # Enter the name of your executable
    )
    ])
