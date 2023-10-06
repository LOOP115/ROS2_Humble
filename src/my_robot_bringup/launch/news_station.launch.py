from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["Allen", "Barry", "Charlie", "Daniel", "Emily"]

    robot_news_station_nodes = []

    for name in robot_names:
        robot_news_station_nodes.append(
            Node(
                package="my_py_pkg",
                executable="robot_news_station",
                remappings=[("__node", f"{name.lower()}_news_station")],
                parameters=[{"name": name}],
            )
        )

    smartphone_node = Node(package="my_py_pkg", executable="smartphone")

    for node in robot_news_station_nodes:
        ld.add_action(node)
    ld.add_action(smartphone_node)
    return ld
