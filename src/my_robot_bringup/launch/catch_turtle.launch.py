from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(package="turtlesim", executable="turtlesim_node")

    turtle_controller_node = Node(
        package="catch_turtle",
        executable="turtle_controller",
        parameters=[{"strategy": "closest"}],
    )

    turtle_spawner_node = Node(
        package="catch_turtle",
        executable="turtle_spawner",
        parameters=[{"hz": 1.25}, {"name_prefix": "prey"}],
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_controller_node)
    ld.add_action(turtle_spawner_node)
    return ld
