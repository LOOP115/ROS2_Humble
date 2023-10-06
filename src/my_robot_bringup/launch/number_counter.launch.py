from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    number_publisher_node = Node(
        package="number_counter",
        executable="number_publisher",
        remappings=[remap_number_topic],
        parameters=[{"num": 5}, {"hz": 0.5}],
    )

    number_counter_node = Node(
        package="number_counter",
        executable="number_counter",
        remappings=[remap_number_topic],
    )

    number_subscriber_node = Node(
        package="number_counter",
        executable="number_subscriber",
        name="counter_subscriber",
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    ld.add_action(number_subscriber_node)
    return ld
