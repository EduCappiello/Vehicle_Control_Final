# Vehicle control Final Exam
This tutorial shows the use of gazebo and ROS for mobile robot simulation
```
System software versions:
Ubuntu version 20.04
ROS1 noetic
Gazebo 11.14.0
```
## Setup

### First, detete this two lines from the ~/.bashrc and install the necesary packages.
```bash
sed -i '/source \/opt\/ros\/foxy\/setup.bash/d' ~/.bashrc
```
```bash
sed -i '/source install\/setup.bash/d' ~/.bashrc
```
Let's install the needed packages:
```bash
sudo apt-get -y install ros-noetic-gazebo-ros-pkgs
```
```bash
sudo apt-get -y install ros-noetic-gazebo-msgs
```
```bash
sudo apt-get -y install ros-noetic-gazebo-plugins
```
```bash
sudo apt-get -y install ros-noetic-teleop-twist-keyboard
```
```bash
sudo apt-get -y install ros-noetic-gazebo-ros-control
```
```bash
sudo apt -y install ros-noetic-xacro
```
```bash
sudo apt -y install ros-noetic-robot-state-publisher
```

### Create the work space and catkin space:

```bash
export ROS_DISTRO=noetic
mkdir -p ~/robot_four/src
cd ~/robot_four
catkin_make
source ~/robot_four/devel/setup.bash
echo $ROS_PACKAGE_PATH
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
Paste the content of this file into "robot.xacro":

[robot.xacro](robot.xacro)

Create the gazebo file:
```bash
gedit robot.gazebo
```

Paste the content of this file into "robot.gazebo":

[robot.gazebo](robot.gazebo)

Build the project:
```bash
cd ~/robot_four
catkin_make
```
### Create the launch file and launch the file in gazebo

```bash
cd ~/robot_four/src/robot_model_pkg
mkdir launch
cd ~/robot_four/src/robot_model_pkg/launch
gedit robot_xacro.launch
```
Paste the content of this file into robot_xacro.launch:

[robot_xacro.launch](robot_xacro.launch)

Save the file

Open a new terminal and start:
```bash
roscore
```
Finally we can launch our model in gazebo as:
```bash
source ~/robot_four/devel/setup.bash
roslaunch robot_model_pkg robot_xacro.launch
```

Create a teleop node for controlling the robot. Open a new terminal and type
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Then, investigate the topics that are being used and published to. Open a new terminal and type:

```bash
rostopic list
rostopic echo /odom
rostopic info /cmd_vel
```
