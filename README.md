# ROS 2 Study Notes



### Setup

* Install Ubuntu 22.04
* Install ROS2 Humble
* [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/index.html)
* [ROS 2 GitHub](https://github.com/ros2)



### Workspace and Package

#### Install the ROS2 build tool - Colcon

* `sudo apt install python3-colcon-common-extensions`
* Add `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash` to `~/.bashrc`

#### Create a ROS2 workspace

* `mkdir <workspace>`
* `mkdir src`
* `colcon build`
* Run `source <workspace>/install/setup.bash` when starting a new terminal or add this line into `~/.bashrc`

#### Create a Python Package

* `cd src`
* `ros2 pkg create <package> --build-type ament_python --dependencies rclpy`
* Add dependencies in `package.xml`
* `colcon build`



### ROS2 Tools

#### Colcon

* Build selected packages
  * `colcon build --packages-select <package>`
* Enable symlink install
  * `colcon build --symlink-install`
  * When you use the `--symlink-install` flag, the built packages are installed using symbolic links (symlinks). This means that instead of copying the package files to the install directory, symlinks are created. Symlinks are pointers to the original files, so changes you make to the source files are immediately reflected in the installed packages without the need to rebuild and reinstall.
* Disable symlink install
  * `colcon build --cmake-clean-first`
* Possible [solutions](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/#400052) to the deprecation warning of `setup.py`

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

* `cd ~/<workspace path>/src/<package>/<package>`
* `touch <node_script>.py`

#### Configure the node

* Enter into `setup.py`
* In `console_scripts`, add `<node> = <package>.<node_script>:main`

```python
def main(args=None):
    rclpy.init(args=args)    # Must be called at the start
    node = Node()    # The node's name can be different from the script's name
    rclpy.spin(node)    # Start the event loop for a node
    rclpy.shutdown()    # Must be called at the end
```

#### 3 ways to run the node

* Executable Python script
  * `cd ~/<workspace path>/src/<package>/<package>`
  * `chmod +x <node_scipt>.py`
  * `./<node_script>.py`
* From the installed file
  * `cd ~/<workspace path>/install/<package>/lib/<package>`
  * `./<node>`
* ROS2 CLI
  * Remember to source the workspace
  * `ros2 run <package> <node>`

#### Node CLI

* List all nodes: `ros2 node list`
* Info of a node: `ros2 node info <node>`
* Remap a node: `ros2 run <package> <node> --ros-args -r __node:=<new_node>`

#### Node Template

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
* Remap a topic: `ros2 run <package> <node> --ros-args -r <topic>:=<new_topic>`



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
* Info of a service: `ros2 service info <service>`
* Type of a service: `ros2 service type <service>`
* Call a service: `ros2 service call <service> <service_type> <request_data>`
* Remap a service: `ros2 run <package> <node> --ros-args -r <service>:=<new_service>`
* Use RQt plugins to call the service



### ROS2 Interfaces

#### `msg` and `srv`

* Create a message definition using `msg` [primitive types](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
* Create a message definition using other message definitions
* Other message definitions can be included in a service
* Services cannot be included into other services

#### Create a package for the custom interface

* `cd <workspace>/src`
* `ros2 pkg create <interface_package>`
* `cd <interface_package>`
* `rm -rf include/ src/`
* Add the following lines into `package.xml`

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

* Add the following line into `CMakeLists.txt  # find dependencies`

```cmake
find_package(rosidl_default_generators REQUIRED)
```

#### Create custom `msg` / `srv`

* In `<interface_package>`: 
  * `mkdir <msg_dir> && cd <msg_dir>`
  * `mkdir <srv_dir> && cd <srv_dir>`

* `touch <msg>.msg` or `touch <srv>.srv`
  * Uppercase first letter
  * Camel case
* Add the following lines into `CMakeLists.txt  # find dependencies`

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "<msg_dir>/<msg>.msg"
  "<srv_dir>/<srv>.srv"
  # Add newly created msg/srv
)
```

#### CLI

* List of interfaces: `ros2 interface list`
* Show info of an interface: `ros2 interface show <interface>`
* Show interfaces in the package: `ros2 interface package <package>`

