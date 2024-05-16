# Vehicle control Final Exam
This tutorial shows the use of gazebo and ROS for mobile robot simulation
```
System software versions:
Ubuntu version 20.04
ROS1 noetic
Gazebo 11.14.0
```
## Setup

### First install the necesary packages.

Let's install the needed packages:
```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs
sudo apt-get install ros-noetic-gazebo-msgs
sudo apt-get install ros-noetic-gazebo-plugins
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-xacro
sudo apt install ros-noetic-robot-state-publisher
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
Paste the following code in the file "robot.xacro":

```xml
<?xml version="1.0"?>

<robot name="differential_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Body dimensions -->
    <xacro:property name="body_link_x_dim" value="1"/>
    <xacro:property name="body_link_y_dim" value="0.6"/>
    <xacro:property name="body_link_z_dim" value="0.3"/>

    <!-- Wheel dimensions-->
    <xacro:property name="wheel_link_radius" value="0.15"/>
    <xacro:property name="wheel_link_length" value="0.1"/>
    <xacro:property name="wheel_link_z_location" value="-0.1"/>

    <!-- Material density -->
    <xacro:property name="body_density" value="2710.0"/>
    <xacro:property name="wheel_density" value="2710.0"/>

    <!-- Pi constant -->
    <xacro:property name="pi_const" value="3.14159265"/>

    <!-- Robot body and wheel mass-->
    <xacro:property name="body_mass" value="${body_density*body_link_x_dim*body_link_y_dim*body_link_z_dim}"/>
    <xacro:property name="wheel_mass" value="${wheel_density*pi_const*wheel_link_radius*wheel_link_radius*wheel_link_length}"/>

    <!-- Moments of inertia of the wheel-->
    <xacro:property name="Iz_wheel" value="${(1/2)*wheel_mass*wheel_link_radius*wheel_link_radius}"/>
    <xacro:property name="I_wheel" value="${(1.0/12.0)*wheel_mass*(3.0*wheel_link_radius*wheel_link_radius+wheel_link_length*wheel_link_length)}"/>

    <!-- This macro defines the complete Inertial section of the wheel-->
    <!-- It is used later in the code -->
    <xacro:macro name="inertia_wheel">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="${wheel_mass}"/>
            <inertia ixx="${I_wheel}" ixy="0.0" ixz="0.0" iyy="${I_wheel}" iyz="0" izz="${Iz_wheel}"/>
        </inertial>
    </xacro:macro>

    <!-- Over here we include the file that defines extra Gazebo options and motion control driver-->
    <xacro:include filename="$(find robot_model_pkg)/urdf/robot.gazebo"/>

    <!--#################################-->
    <!--FROM HERE WE DEFINE LINKS, JOINTS-->
    <!--#################################-->

    <!-- We need to have this dummy link otherwise Gazebo will complain-->
    <link name="dummy">
    </link>
    <joint name="dummy_joint" type="fixed">
        <parent link="dummy"/>
        <child link="body_link"/>
    </joint>

    <!--####################################-->
    <!--#   START: Body link of the robot  #-->
    <!--####################################-->

    <link name="body_link">
        <visual>
            <geometry>
                <box size="${body_link_x_dim} ${body_link_y_dim} ${body_link_z_dim}" />
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0"/>
        </visual>
        <collision>
            <geometry>
                <box size="${body_link_x_dim} ${body_link_y_dim} ${body_link_z_dim}"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0"/>
        </collision>
        <inertial>
            <mass value="${body_mass}"/>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <inertia ixx="${(1/12)*body_mass*(body_link_y_dim*body_link_y_dim+body_link_z_dim*body_link_z_dim)}" 
                     ixy="0" 
                     ixz="0" 
                     iyy="${(1/12)*body_mass*(body_link_x_dim*body_link_x_dim+body_link_z_dim*body_link_z_dim)}" 
                     iyz="0" 
                     izz="${(1/12)*body_mass*(body_link_y_dim*body_link_y_dim+body_link_x_dim*body_link_x_dim)}"/>
        </inertial>
    </link>

    <!-- ###################################-->
    <!-- #   END:Body link of the robot    #-->
    <!-- ###################################-->


    <!-- ###########################################################-->
    <!-- #   START: Back right wheel of the robot and the joint    #-->
    <!-- ###########################################################-->

    <joint name="wheel1_joint" type="continuous">
        <parent link="body_link"/>
        <child link="wheel1_link" />
        <origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${-body_link_y_dim/2-wheel_link_length/2} 
        ${wheel_link_z_location}" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
        <limit effort="1000" velocity="1000"/>
        <dynamics damping="1.0" friction="1.0"/>
    </joint>

    <link name="wheel1_link">
        <visual>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </visual>
        <collision>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </collision>
        <xacro:inertia_wheel/>
    </link>

    <!-- ###########################################################--> 
    <!-- #   END: Back right wheel of the robot and the joint      #-->
    <!-- ###########################################################-->

    <!-- ###########################################################--> 
    <!-- #   Start: Back left wheel of the robot and the joint     #-->
    <!-- ###########################################################-->

    <joint name="wheel2_joint" type="continuous">
        <parent link="body_link"/>
        <child link="wheel2_link" />
        <origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${body_link_y_dim/2+wheel_link_length/2} 
        ${wheel_link_z_location}" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
        <limit effort="1000" velocity="1000"/>
        <dynamics damping="1.0" friction="1.0"/>
    </joint>

    <link name="wheel2_link">
        <visual>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </visual>
        <collision>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </collision>
        <xacro:inertia_wheel/>
    </link>
    <!-- ###########################################################--> 
    <!-- #   END: Back left wheel of the robot and the joint       #-->
    <!-- ###########################################################-->

    <!-- ###########################################################-->
    <!-- #   START: Front right wheel of the robot and the joint   #-->
    <!-- ###########################################################-->

    <joint name="wheel3_joint" type="continuous">
        <parent link="body_link"/>
        <child link="wheel3_link" />
        <origin xyz="${body_link_x_dim/2-1.2*wheel_link_radius} ${-body_link_y_dim/2-wheel_link_length/2} 
        ${wheel_link_z_location}" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
        <limit effort="1000" velocity="1000"/>
        <dynamics damping="1.0" friction="1.0"/>
    </joint>

    <link name="wheel3_link">
        <visual>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </visual>
        <collision>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </collision>
        <xacro:inertia_wheel/>
    </link>

    <!-- ###########################################################-->
    <!-- #   END: Front right wheel of the robot and the joint     #-->
    <!-- ###########################################################-->

    <!-- ###########################################################-->
    <!-- #   START: Front left wheel of the robot and the joint    #-->
    <!-- ###########################################################-->

    <joint name="wheel4_joint" type="continuous">
        <parent link="body_link"/>
        <child link="wheel4_link" />
        <origin xyz="${body_link_x_dim/2-1.2*wheel_link_radius} ${body_link_y_dim/2+wheel_link_length/2} 
        ${wheel_link_z_location}" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
        <limit effort="1000" velocity="1000"/>
        <dynamics damping="1.0" friction="1.0"/>
    </joint>

    <link name="wheel4_link">
        <visual>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </visual>
        <collision>
            <origin rpy="1.570795 0 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
            </geometry>
        </collision>
        <xacro:inertia_wheel/>
    </link>

    <!-- ###########################################################-->
    <!-- #   END: Front left wheel of the robot and the joint      #-->
    <!-- ###########################################################-->
</robot>

```
Create the gazebo file:
```bash
gedit robot.gazebo
```

Paste the following code in the file "robot.gazebo":

```xml
<?xml version="1.0"?>
<robot>
    <!-- Everything is described here -->
    <!-- http://classic.gazebosim.org/tutorials?tut=ros.urdf&cat=connect_ros -->
    <!-- mu1 and mu2 are friccion coefficients-->
    <gazebo reference="body_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Red</material>
    </gazebo>
    
    <gazebo reference="wheell_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
    </gazebo>
    
    <gazebo reference="wheel2_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
    </gazebo>

    <gazebo reference="wheel3_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
    </gazebo>
    
    <gazebo reference="wheel4_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
    </gazebo>
    
    <!-- Controller for the 4-wheeled robot -->
    <gazebo>
        <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">
            <!-- Control update rate in Hz -->
            <updateRate>100.0</updateRate>
            <!-- Leave this empty otherwise, there will be problems with sending control commands -->
            <robotNamespace></robotNamespace>
            <!-- Robot kinematics -->
            <leftFrontJoint>wheel4_joint</leftFrontJoint>
            <rightFrontJoint>wheel3_joint</rightFrontJoint>
            <leftRearJoint>wheel2_joint</leftRearJoint>
            <rightRearJoint>wheel1_joint</rightRearJoint>
            <wheelSeparation>${body_link_y_dim+wheel_link_length}</wheelSeparation>
            <wheelDiameter>${2*wheel_link_radius}</wheelDiameter>
            <!-- Maximum torque which the wheels can produce, in Nm, defaults to 5 Nm -->
            <torque>1000</torque>
            <!-- Topic to receive geometry_msgs/Twist message commands, defaults to 'cmd_vel' -->
            <commandTopic>cmd_vel</commandTopic>
            <!-- Topic to publish nav_msgs/Odometry messages, defaults to 'odom' -->
            <odometryTopic>odom</odometryTopic>
            <!-- Odometry frame, defaults to 'odom' -->
            <odometryFrame>odom</odometryFrame>
            <!-- Robot frame to calculate odometry from, defaults to 'base_footprint' -->
            <robotBaseFrame>dummy</robotBaseFrame>
            <!-- Set to true to publish transforms for the wheel links, defaults to false -->
            <publishWheelTF>true</publishWheelTF>
            <!-- Set to true to publish transforms for the odometry, defaults to true -->
            <publishOdom>true</publishOdom>
            <!-- Set to true to publish sensor_msgs/JointState on /joint_states for the wheel joints, defaults to false -->
            <publishWheelJointState>true</publishWheelJointState>
            <!-- This part is required by Gazebo -->
            <covariance_x>0.0001</covariance_x>
            <covariance_y>0.0001</covariance_y>
            <covariance_yaw>0.01</covariance_yaw>
        </plugin>
    </gazebo>
</robot>
```
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
Paste the following code in robot_xacro.launch:
```xml
<?xml version="1.0"?>

<launch>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="true"/>
        <arg name="headless" value="false"/>
        <arg name="debug" value="false"/>
    </include>

    <!-- Load the robot description -->
    <param name="robot_description" command="$(find xacro)/xacro '$(find robot_model_pkg)/urdf/robot.xacro'"/>

    <!-- Robot state publisher node -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

    <!-- Spawn the model -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -model robot_model -param robot_description" />

</launch>
```
Save the file

Open a new terminal and start:
```bash
roscore
```
Finally we can launch our model in gazebo as:

```bash
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