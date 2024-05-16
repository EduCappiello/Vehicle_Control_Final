# Vehicle control Final Exam
This tutorial shows the use of gazebo and ROS for mobile robot simulation
```
System software versions:
Ubuntu version 22.04
ROS2 Humble
Gazebo Fortress
```
## Setup

### First install the necesary packages.

Let's install the needed packages:
```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-msgs
sudo apt-get install ros-humble-gazebo-plugins
sudo apt-get install ros-humble-teleop-twist-keyboard
```

### Create the work space and catkin space:

```bash
mkdir -p ~/robot_four/src
cd ~/robot_four
catkin_make
source ~/robot_four/devel/setup.bash
echo $ROS PACKAGE_PATH
cd ~/robot_four/src
```

Create the package:
```bash
catkin_create_pkg robot_model_pkg gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control
```
Dependencies are:
- gazebo_msgs
- gazebo_plugins
- gazebo_ros
- gazebo_ros_control


### Create the source files defining the robot geometry
```bash
cd ~/robot_four/src/robot_model_pkg
mkdir urdf
cd ~/robot_four/src/robot_model_pkg/urdf
gedit robot.xacro
```
Check the URDFfile:



You can also create subheadings by using more `#` symbols.

- List item 1
- List item 2
- List item 3

You can create bulleted lists using `-` or `*` symbols.

1. Numbered item 1
2. Numbered item 2
3. Numbered item 3

You can create numbered lists by using numbers followed by a `.`.

**Bold text** and *italic text* can be created using double and single asterisks, respectively.

[Link to a website](https://www.example.com)

You can create links by enclosing the link text in square brackets and the URL in parentheses.

![Image](path/to/image.jpg)

You can also add images by using an exclamation mark followed by the image path.

> Blockquote

You can create blockquotes using the `>` symbol.

`Inline code`

You can highlight inline code by enclosing it in backticks.

