# [ROS2 For Beginners](https://www.udemy.com/course/ros2-for-beginners/)



### Setup

* Install Ubuntu 22.04
* Install ROS2 Humble
  * [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/index.html)
  * Add `source /opt/ros/humble/setup.bash` to `~/.bashrc`
  * `pip install catkin_pkg empy lark pytest jinja2 pyaml typeguard`



### Workspace and Package

#### Install the ROS2 build tool - Colcon

* `sudo apt install python3-colcon-common-extensions`
* Add `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash` to `~/.bashrc`

#### Create a ROS2 workspace

* `mkdir <workspace>`
* `mkdir src`
* `colcon build`
* Run `source <workspace>/install/setup.bash` when starting a new terminal or add this line into `~/.bashrc`

#### Create a Python package

* `cd src`
* `ros2 pkg create <pkg> --build-type ament_python --dependencies rclpy`
* Add dependencies in `package.xml`
* `cd .. && colcon build`



### ROS2 Tools

#### Colcon

* Build selected packages
  * `colcon build --packages-select <pkg>`
* Enable symlink install
  * `colcon build --symlink-install`
  * When you use the `--symlink-install` flag, the built packages are installed using symbolic links (symlinks). This means that instead of copying the package files to the install directory, symlinks are created. Symlinks are pointers to the original files, so changes you make to the source files are immediately reflected in the installed packages without the need to rebuild and reinstall.
* Disable symlink install
  * `colcon build --cmake-clean-first`
* Allow overriding
  * `colcon build --allow-overriding`

* Possible [solutions](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/#400052) to the deprecation warning of `setup.py`
  * Downgrade the `setuptools` to `58.2.0` in `dist-packages`
  * `sudo pip3 install --target=/usr/lib/python3/dist-packages setuptools==58.2.0 --upgrade`
  * `pip install setuptools==58.2.0`


#### RQt

* `rqt` (short for ROS Qt) is a framework and set of tools in ROS2 that provides a GUI for various tasks.

* `rqt_graph`



### ROS2 Nodes

#### Definition

**Nodes are fundamental units of computation that perform various tasks within a robotic system.**

* Subprograms in the application, responsible for only one thing
* Combined into a graph
* Communicate with each other through topic, services, and parameters
* Reduce code complexity
* Fault tolerance

#### Create a node

* `cd ~/<workspace>/src/<pke>/<pkg>`
* `touch <node>.py`
* `chmod +x <node>.py`

#### Configure the node

* Enter into `setup.py`
* In `console_scripts`, add `<node> = <pkg>.<node>:main`

```python
def main(args=None):
    rclpy.init(args=args)    # Must be called at the start
    node = Node()    # The node's name can be different from the script's name
    rclpy.spin(node)    # Start the event loop for a node
    rclpy.shutdown()    # Must be called at the end
```

#### 3 ways to run the node

* Executable Python script
  * `cd ~/<workspace>/src/<pkg>/<pkg>`
  * `./<node>.py`
* From the installed file
  * `cd ~/<workspace>/install/<pkg>/lib/<pkg>`
  * `./<node>`
* **ROS2 CLI**
  * Remember to source the workspace
  * `ros2 run <pkg> <node>`

#### Node CLI

* List all nodes: `ros2 node list`
* Info of a node: `ros2 node info <node>`
* Remap a node: `ros2 run <pkg> <node> --ros-args -r __node:=<new_node>`

#### [Template](src/my_py_pkg/my_py_pkg/template_node.py)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
 
 
class MyCustomNode(Node):
    def __init__(self):
        super().__init__("node_name")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Node()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
```



### ROS2 Topics

#### Definition

**A topic is a named bus over which nodes exchange messages.**

* Unidirectional data stream
* Anonymous
* A topic has a message type
* A node can have many publishers/subscribers for many topics

**Elements**

* Name
* `msg` definition

#### Topic CLI

* List all topics: `ros2 topic list`
* Info of a topic: `ros2 topic info <topic>`
* Publish from the terminal: `ros2 topic pub -r <rate> <topic> <msg_type> <msg>`
* Subscribe from the terminal: `ros2 topic echo <topic>`
* Frequency: `ros2 topic hz <topic>`
* Bandwidth: `ros2 topic bw <topic>`
* Remap a topic: `ros2 run <pkg> <node> --ros-args -r <topic>:=<new_topic>`



### ROS2 Services

#### Definition

**A ROS2 Service is a client/server system.**

* Synchronous or asynchronous
* One message type for Request, one message type for Response
* A service server can only exist once, but can have many clients

**Elements**

* Name
* `srv` definition

#### Service CLI

* List all services: `ros2 service list`
* Type of a service: `ros2 service type <service>`
* Call a service: `ros2 service call <service> <service_type> <request_data>`
* Remap a service: `ros2 run <pkg> <node> --ros-args -r <service>:=<new_service>`
* Use RQt plugins to call the service



### ROS2 Interfaces

#### `msg` and `srv`

* Create a message definition using `msg` [primitive types](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
* Create a message definition using other message definitions
* Other message definitions can be included in a service
* Services cannot be included into other services

#### Create a package for the custom interface

* `cd <workspace>/src`
* `ros2 pkg create <interface_pkg>`
* `cd <interface_pkg>`
* `rm -rf include/ src/`
* Add the following lines into [`package.xml`](src/my_robot_interfaces/package.xml)

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

* Add the following line into [`CMakeLists.txt`](src/my_robot_interfaces/CMakeLists.txt)

```cmake
find_package(rosidl_default_generators REQUIRED)
```

#### Create custom `msg` / `srv`

* In `<interface_pkg>`: 
  * `mkdir <msg_dir> && cd <msg_dir>`
  * `mkdir <srv_dir> && cd <srv_dir>`

* `touch <msg>.msg` or `touch <srv>.srv`
  * Uppercase first letter
  * Camel case
* Add the following lines into [`CMakeLists.txt`](src/my_robot_interfaces/CMakeLists.txt)

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "<msg_dir>/<msg>.msg"
  "<srv_dir>/<srv>.srv"
  # Add newly created msg/srv
)
```

#### Interface CLI

* List of interfaces: `ros2 interface list`
* Show info of an interface: `ros2 interface show <interface>`
* Show interfaces in the package: `ros2 interface package <pkg>`



### ROS2 Parameters

#### Definition

* Settings for your nodes, value set at run time
* A Parameter is specific to a node

#### Parameter CLI

* List of parameters: `ros2 param list`
* Get parameter value: `ros2 param get <node> <param>`
* Declare the parameter: `ros2 run <pkg> <node> --ros-args -p <param>:=<value>`



### ROS2 Launch Files

#### Create a package for launch files

* `cd <workspace>/src`
* `ros2 pkg create <robot>_bringup`
* `rm -rf include/ src/`
* `mkdir launch`
* Add the following lines into [`CMakeLists.txt`](src/my_robot_bringup/CMakeLists.txt)

```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

#### Create a launch file

* `cd launch`
* `touch <app>.launch.py`
* `chmod +x <app>.launch.py`
* [Template](src/my_robot_bringup/launch/template.launch.py)

```python
from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    return ld
```

* Add depended packages in [`package.xml`](src/my_robot_bringup/package.xml)

```xml
<exec_depend>pkg</exec_depend>
```

* Launch: `ros2 launch <robot>_bringup <app>.launch.py`



### ROS2 Bags

#### Definition

In ROS 2, a "bag" is a file format and tool for recording, storing, and playing back ROS data, including messages, topics, and other ROS-related information.

#### Create a bag

* Choose a location for storing bags and `mkdir bags && cd bags`
* Record topics to a bag: `ros2 bag record <topic1> <topic2> -o <bag>`
* Record all topics: `ros2 bag record -a -o <bag>`
* Info of a bag: `ros2 bag info <bag>`
* Play a bag: `ros2 bag play <bag>`

