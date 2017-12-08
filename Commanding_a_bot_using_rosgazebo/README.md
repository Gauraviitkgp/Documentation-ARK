# Commanding_a_bot
Firstly install ros_desktop_full. Later add on

    sudo apt-get install ros-indigo-joint-state-controller
    sudo apt-get install ros-indigo-effort-controllers
    sudo apt-get install ros-indigo-position-controllers

Then create a ros workspace. Let it be named catkin_ws. In the src folder of the workspace do the following task
[It loads a sample package named rrbot]

    cd ~/catkin_ws/src/
    git clone https://github.com/ros-simulation/gazebo_ros_demos.git
    cd ..
    catkin_make

To launch rrbot in gazebo do the following
    roslaunch rrbot_gazebo rrbot_world.launch

Launch files would be discussed seperately later. To examine the code of rrbot. Go to rrbot_description and open rrbot.xacro.

Tags used in the file is being discussed later.

# Ros Control
1) Add the gazebo_ros_control plugin into your file

The default plugin XML should be added to your URDF:

    <gazebo>
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/MYROBOT</robotNamespace>
      </plugin>
    </gazebo>

We add a <transmission> block similar to the following for every joint that we wish to have Gazebo actuate.  Open your rrbot.xacro file and at the bottom of the file you should see:

    <transmission name="tran1">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="joint1">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="motor1">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>

    <transmission name="tran2">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="joint2">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="motor2">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>

There are 3 types of hardwareInterface:-    hardware_interface::JointStateInterface,    hardware_interface::EffortJointInterface
,   hardware_interface::VelocityJointInterface.

(tough in src file everything is already created, we'll start from scratch )

Firstly create a package(here MYROBOT=rrbot)

    mkdir ~/catkin_ws
    cd ~/catkin_ws
    catkin_create_pkg MYROBOT_control ros_control ros_controllers
    cd MYROBOT_control
    mkdir config
    mkdir launch

## Create a yaml file
on MYROBOT_control/config/rrbot_control.yaml. For Rrbot it is as:-

     <!- Publish all joint states -->
      joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 50  

      <!- Position Controllers -->
      joint1_position_controller:
        type: effort_controllers/JointPositionController
        joint: joint1
        pid: {p: 100.0, i: 0.01, d: 10.0}
      joint2_position_controller:
        type: effort_controllers/JointPositionController
        joint: joint2
        pid: {p: 100.0, i: 0.01, d: 10.0}

## Create a roslaunch files
MYROBOT_control/launch/MYROBOT_control.launch. For Rrbot:-

    <launch>

    <!-- Load joint controller configurations from YAML file to parameter server -->
    <rosparam file="$(find rrbot_control)/config/rrbot_control.yaml" command="load"/>

    <!-- load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
      output="screen" ns="/rrbot" args="joint1_position_controller joint2_position_controller joint_state_controller"/>

    <!-- convert joint states to TF transforms for rviz, etc -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
      respawn="false" output="screen">
      <remap from="/joint_states" to="/rrbot/joint_states" />
    </node>

    </launch>

For furthur details on roslaunch:- http://gazebosim.org/tutorials?tut=ros_roslaunch

Now start rrbot in gazebo by

Start the RRBot simulation:

       roslaunch rrbot_gazebo rrbot_world.launch
       
Load the controllers for the two joints by running the second launch file:

    roslaunch rrbot_control rrbot_control.launch

For testing send example joint commands:-

        rostopic pub -1 /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 1.5"
        rostopic pub -1 /rrbot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"

And see in gazebo. You'll notice that the position changes.


Tags used in the files:-
## Links
The link element describes a rigid body with an inertia, visual features,
eg of link element

    <link name="my_link">
      <inertial>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <mass value="1"/>
        <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
      </inertial>

      <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <box size="1 1 1" />
        </geometry>
        <material name="Cyan">
          <color rgba="0 1.0 1.0 1.0"/>
        </material>
      </visual>    
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="1" length="0.5"/>
        </geometry>
      </collision>
    </link>

 \<inertial\> (optional):    The inertial properties of the link.
    
     <mass>: The mass of the link is represented by the value attribute of this element
     <inertia>: The 3x3 rotational inertia matrix, represented in the inertia frame. Because the rotational inertia matrix is symmetric, only 6 above-diagonal elements of this matrix are specified here, using the attributes ixx, ixy, ixz, iyy, iyz, izz.

\<visual\> (optional)  The visual properties of the link

    name (optional):  Specifies a name for a part of a link's geometry.
    <origin> (optional: defaults to identity if not specified):The reference frame of the visual element with respect to the reference frame of the li. The origin tag represents the center of mass of this link.nk.
    xyz (optional: defaults to zero vector): Represents the $$x,y,z$$ offset.
    rpy (optional: defaults to identity if not specified):Represents the fixed axis roll, pitch and yaw angles in radians.

    <geometry> (required): The shape of the visual object. This can be one of the following:
    <box>
    size attribute contains the three side lengths of the box. The origin of the box is in its center.
    <cylinder>
    Specify the radius and length. The origin of the cylinder is in its center. cylinder_coordinates.png
    <sphere>
    Specify the radius. The origin of the sphere is in its center.
    <mesh>
    A trimesh element specified by a filename, and an optional scale that scales the mesh's axis-aligned-bounding-box.

    <material> (optional):The material of the visual element. It is allowed to specify a material element outside of the 'link' object
    <color> (optional):rgba The color of a material specified by set of four numbers representing red/green/blue/alpha, each in the range of [0,1].
    <texture> (optional)
    The texture of a material is specified by a filename

 \<collision\> (optional):The collision properties of a link.
  name (optional):Specifies a name for a part of a link's geometry. This is useful to be able to refer to specific bits of the geometry of a link.
      
      <origin> (optional: defaults to identity if not specified):The reference frame of the collision element, relative to the reference frame of the link.
      xyz (optional: defaults to zero vector)
      Represents the $$x,y,z$$ offset.
      rpy (optional: defaults to identity if not specified)
      Represents the fixed axis roll, pitch and yaw angles in radians.

For furthur details on link visit:- http://wiki.ros.org/urdf/XML/link#CA-3f2893626f14a784b3d26cf27078c62b79d36f23_1

## Joints
The joint element describes the kinematics and dynamics of the joint and also specifies the safety limits of the joint.eg

    <joint name="my_joint" type="floating">
        <origin xyz="0 0 1" rpy="0 0 3.1416"/>
        <parent link="link1"/>
        <child link="link2"/>

        <calibration rising="0.0"/>
        <dynamics damping="0.0" friction="0.0"/>
        <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
        <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
     </joint>

The joint element has two attributes:
   
    name (required):Specifies a unique name of the joint

    type (required):Specifies the type of joint, where type can be one of the following:
        revolute - a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
    
        continuous - a continuous hinge joint that rotates around the axis and has no upper and lower limits
    
        prismatic - a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
    
        fixed - This is not really a joint because it cannot move. All degrees of freedom are locked. This type of joint does not require the axis, calibration, dynamics, limits or safety_controller.
    
        floating - This joint allows motion for all 6 degrees of freedom.
    
        planar - This joint allows motion in a plane perpendicular to the axis.

elements of joint :-
    
    <origin> (optional: defaults to identity if not specified)
    This is the transform from the parent link to the child link. The joint is located at the origin of the child link.
    xyz (optional: defaults to zero vector):Represents the $$x,y,z$$ offset.
    rpy (optional: defaults 'to zero vector 'if not specified):
    Represents the rotation around fixed axis: first roll around x, then pitch around y and finally yaw around z. All angles are specified in radians.
    
    <parent> (required):Parent link name with mandatory attribute:
    link
    The name of the link that is the parent of this link in the robot tree structure.
    
    <child> (required): Child link name with mandatory attribute:
    link
    The name of the link that is the child link.
    
    <axis> (optional: defaults to (1,0,0))
    The joint axis specified in the joint frame. This is the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints. The axis is specified in the joint frame of reference. Fixed and floating joints do not use the axis field.
    
    xyz (required)
    Represents the $$x,y,z$$ components of a vector. The vector should be normalized.
For furthur details visit
http://wiki.ros.org/urdf/XML/joint
