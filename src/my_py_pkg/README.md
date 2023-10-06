## Basic ROS2 with Python

* Simple node
  * `ros2 run my_py_pkg py_node`
* Publish / Subscribe
  * `ros2 run my_py_pkg robot_news_station --ros-args -p name:="<name>"`
  * `ros2 run my_py_pkg smartphone`
* Server / Client
  * `ros2 run my_py_pkg add_two_ints_server`
  * `ros2 run my_py_pkg add_two_ints_client`
* Custom `msg`
  * `ros2 run my_py_pkg hw_status_publisher`
  * `ros2 topic echo /hardware_status`

