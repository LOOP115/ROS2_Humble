# ROS2 Study Notes



#### Environment Setup

* Install Ubuntu 22.04
* Install ROS2 Humble
  * [Documentation](https://docs.ros.org/en/humble/index.html)



#### Workspace and Package

* **Install the ROS2 build tool - Colcon**
  * `sudo apt install python3-colcon-common-extensions`
  * Add `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash` to `~/.bashrc`

* **Create a ROS2 workspace**
  * `mkdir <workspace>`
  * `mkdir src`
  * Add `source <workspace>/install/setup.bash` to `~/.bashrc`

* **Create a Python Package**
  * `ros2 pkg create <package> --build-type ament_python --dependencies rclpy`
  * Add dependencies in `package.xml`
  * `colcon build`



#### ROS2 Nodes

Nodes are fundamental units of computation that perform various tasks within a robotic system.

* Subprograms in the application, responsible for only one thing
* Combined into a graph
* Communicate with each other through topic, services, and parameters
* Reduce code complexity
* Fault tolerance

**Create a node**

* `cd ~/<workspace path>/src/<package>/<package>`
* `touch <node_script>.py`

**Configure the node**

* Enter into `setup.py`
* In `console_scripts`, add `<node> = <package>.<node_script>:main`

```python
def main(args=None):
    rclpy.init(args=args)    # Must be called at the start
    node = Node()    # The node's name can be different from the script's name
    rclpy.spin(node)    # Start the event loop for a node
    rclpy.shutdown()    # Must be called at the end
```

**Run the node**

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

##### Node Template

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



#### ROS2 Tools

##### ROS2 CLI

* List all nodes
  * `ros2 node list`
* Info of a node
  * `ros2 node info <node>`
* Remap a node
  * `ros2 run <package> <node> --ros-args -r __node:=<new_node>`

##### Colcon

* Build selected packages
  * `colcon build --packages-select <package>`
* Enable symlink install
  * `colcon build --symlink-install`
  * When you use the `--symlink-install` flag, the built packages are installed using symbolic links (symlinks). This means that instead of copying the package files to the install directory, symlinks are created. Symlinks are pointers to the original files, so changes you make to the source files are immediately reflected in the installed packages without the need to rebuild and reinstall.
* Disable symlink install
  * `colcon build --cmake-clean-first`
* Possible [solutions](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/#400052) to the deprecation warning of `setup.py`

##### RQt

* `rqt_graph`



#### ROS2 Topics

A topic is a named bus over which nodes exchange messages

* Unidirectional data stream
* Anonymous
* A topic has a message type
* A node can have many publishers/subscribers for many topics

Example message types for topics

* `example_interfaces.msg`

* `geometry_msgs.msg`

* Message type: `ros2 interface show <msg>`

CLI

* List all topics: `ros2 topic list`
* Info of a topic: `ros2 topic info <topic>`
* Publish from the terminal: `ros2 topic pub -r <rate> <topic> <msg_type> <msg>`
* Subscribe from the terminal: `ros2 topic echo <topic>`
* Frequency: `ros2 topic hz <topic>`
* Bandwidth: `ros2 topic bw <topic>`
* Remap a topic: `ros2 run <package> <node> --ros-args -r <topic>:=<new_topic>`



#### ROS2 Services


