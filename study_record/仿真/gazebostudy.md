# study

## robot

The root element in a robot description file must be a **robot**, with all other elements must be encapsulated within.

机器人描述文件的根元素必须是 `<robot>`，所有其他元素都必须封装在其中

![](/home/maple/笔记/images/2025-09-05-13-12-17-urdfrobot.png)

主文件必须包含一个 `name`属性。在包含（引入）的文件中，`name`属性是可选的。如果某个额外包含的文件中也指定了该属性，则其值必须与主文件中的值相同。

```xml
   1 <robot name="pr2">
   2   <!-- pr2 robot links and joints and more -->
   3 </robot>
```

## sensor

The sensor element describes basic properties of a visual sensor

Here is an example of a camera sensor element:

```xml
   1  <sensor name="my_camera_sensor" update_rate="20">
   2    <parent link="optical_frame_link_name"/>
   3    <origin xyz="0 0 0" rpy="0 0 0"/>
   4    <camera>
   5      <image width="640" height="480" hfov="1.5708" format="RGB8" near="0.01" far="50.0"/>
   6    </camera>
   7  </sensor>
```

And below is an example of a laser scan (ray) sensor element:

```xml
   1  <sensor name="my_ray_sensor" update_rate="20">
   2    <parent link="optical_frame_link_name"/>
   3    <origin xyz="0 0 0" rpy="0 0 0"/>
   4    <ray>
   5      <horizontal samples="100" resolution="1" min_angle="-1.5708" max_angle="1.5708"/>
   6      <vertical samples="1" resolution="1" min_angle="0" max_angle="0"/>
   7    </ray>
   8  </sensor>
```

Attributes

- **name** *(required) (string)*
  
  - The name of the link itself

- **update_rate** *(optional) (float) (Hz)*
  
  - The frequency at which the sensor data is generated. If left unspecified, the sensor will generate data every cycle.

elements

**<parent>** *(required)*

- **link** *(required) (string)*
  
  - The name of the link this sensor is attached to.

**<origin>** *(optional: defaults to identity if not specified)*

- This is the pose of the sensor optical frame, relative to the sensor parent reference frame. The sensor optical frame adopts the conventions of z-forward, x-right and y-down.

- **xyz** *(optional: defaults to zero vector)*

- **rpy** *(optional: defaults to identity if not specified)*
  
  - Represents the fixed axis roll, pitch and yaw angles in radians.

![](/home/maple/笔记/images/2025-09-05-20-27-03-urdf_camera.png)

![](/home/maple/笔记/images/2025-09-05-20-27-06-urdf_lidar.png)

Recommended Camera or Ray Resolution

在仿真中，大型传感器会降低整体性能。根据所需的更新频率，建议尽可能将相机或射线分辨率及更新频率保持在较低水平。

## link

```xml
   1  <link name="my_link">
   2    <inertial>
   3      <origin xyz="0 0 0.5" rpy="0 0 0"/>
   4      <mass value="1"/>
   5      <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
   6    </inertial>
   7 
   8    <visual>
   9      <origin xyz="0 0 0" rpy="0 0 0" />
  10      <geometry>
  11        <box size="1 1 1" />
  12      </geometry>
  13      <material name="Cyan">
  14        <color rgba="0 1.0 1.0 1.0"/>
  15      </material>
  16    </visual>
  17 
  18    <collision>
  19      <origin xyz="0 0 0" rpy="0 0 0"/>
  20      <geometry>
  21        <cylinder radius="1" length="0.5"/>
  22      </geometry>
  23    </collision>
  24  </link>
```

![](/home/maple/笔记/images/2025-09-05-20-44-18-inertial.png)

attributes

- **name** *(required)*
  
  - The name of the link itself.

elements

![](/home/maple/笔记/images/2025-09-05-20-51-27-link1.png)

![](/home/maple/笔记/images/2025-09-05-20-51-39-link2.png)

![](/home/maple/笔记/images/2025-09-05-20-51-43-link3.png)

![](/home/maple/笔记/images/2025-09-05-20-51-47-link4.png)

​**​URDF 官方只支持为每个机器人部件（link）定义一组 `<collision>`碰撞体，目的是简化与标准化机器人描述。如果需要更粗糙或额外用途的碰撞几何（如控制器专用），应该使用自定义 XML 标签（如 `<collision_checking>`），而不是依赖 URDF 原生多碰撞组支持。​**​

```xml
 <link name="torso">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/base_link.DAE"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="-0.065 0 0.0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/base_link_simple.DAE"/>
      </geometry>
    </collision>
    <collision_checking>
      <origin rpy="0 0 0" xyz="-0.065 0 0.0"/>
      <geometry>
        <cylinder length="0.7" radius="0.27"/>
      </geometry>
    </collision_checking>
    <inertial>
      ...
    </inertial>
  </link>  
```

## transmissions

> ​**​`<transmission>`元素是 URDF 机器人描述模型的一个扩展​**​，用于描述 ​**​执行器（actuator）与关节（joint）之间的关系​**​。
> 
> 通过它，可以建模一些概念，比如 ​**​齿轮比（gear ratios）​**​ 和 ​**​并联机构（parallel linkages）​**​。
> 
> 一个 ​**​transmission（传动系统）会对力/流量变量进行转换，使得它们的乘积——功率（power）——保持恒定​**​。
> 
> 通过复杂的传动系统，​**​多个执行器可以与多个关节相互关联​**​。

```xml
   1 <transmission name="simple_trans">
   2   <type>transmission_interface/SimpleTransmission</type>
   3   <joint name="foo_joint">
   4     <hardwareInterface>EffortJointInterface</hardwareInterface>
   5   </joint>
   6   <actuator name="foo_motor">
   7     <mechanicalReduction>50</mechanicalReduction>
   8     <hardwareInterface>EffortJointInterface</hardwareInterface>
   9   </actuator>
  10 </transmission>
```

attributes

The transmission element has one attribute:

- **name** (required)
  
  - Specifies the unique name of a transmission

![](/home/maple/笔记/images/2025-09-05-21-00-13-usdf_transmissions.png)

目前，只有 ​**​ros_control​**​ 项目在使用这种 ​**​transmission 元素​**​。开发一种可扩展至所有应用场景的 ​**​更新版传动格式​**​ 是一项复杂的任务，相关内容可在此查阅。

## joint

**​joint 元素​**​ 描述了关节的 ​**​运动学与动力学特性​**​，并规定了该关节的 ​**​安全限制范围​**​。

```xml
   1  <joint name="my_joint" type="floating">
   2     <origin xyz="0 0 1" rpy="0 0 3.1416"/>
   3     <parent link="link1"/>
   4     <child link="link2"/>
   5 
   6     <calibration rising="0.0"/>
   7     <dynamics damping="0.0" friction="0.0"/>
   8     <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
   9     <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
  10  </joint>
```

attributes

The joint element has two attributes:

- **name** *(required)*
  
  - Specifies a unique name of the joint
  
  **type** *(required)*
  
  - Specifies the type of joint, where type can be one of the following:
    
    - **revolute** — a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
    
    - **continuous** — a continuous hinge joint that rotates around the axis and has no upper and lower limits.
    
    - **prismatic** — a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
    
    - **fixed** — this is not really a joint because it cannot move. All degrees of freedom are locked. This type of joint does not require the **<axis>**, **<calibration>**, **<dynamics>**, **<limits>** or **<safety_controller>**.
    
    - **floating** — this joint allows motion for all 6 degrees of freedom.
    
    - **planar** — this joint allows motion in a plane perpendicular to the axis.

elements

[Making sure you&#39;re not a bot!](https://wiki.ros.org/urdf/XML/joint)

**​机器人状态发布器（Robot State Publisher）​**​ 能帮助你将机器人的状态广播到 ​**​TF（坐标变换库）​**​ 中。该工具内部维护了一个机器人的运动学模型，因此只要提供机器人的关节位置（joint positions），它就能计算并发布机器人每个连杆（link）的三维位姿（3D pose）

示例

```xml
  <launch>
   <!-- Load the urdf into the parameter server. -->
   <param name="my_robot_description" textfile="$(find mypackage)/urdf/robotmodel.xml"/>

   <node pkg="robot_state_publisher" type="robot_state_publisher" name="rob_st_pub" >
      <remap from="robot_description" to="my_robot_description" />
      <remap from="joint_states" to="different_joint_states" />
    </node>
  </launch>
```

## 从 URDF 文件创建 KDL 运动学树（KDL Tree）

**​KDL 运动学树是基于 KDL 库构建的一种数据结构，它将 URDF 中描述的机器人模型转换为一种可供运动学和动力学算法使用的分层树状结构。​**​

在这个树中：

- •
  
  每个 ​**​Link（连杆）​**​ 对应 KDL 中的一个 ​**​Segment（段）​**​；

- •
  
  每个 ​**​Joint（关节）​**​ 也对应到 KDL Segment 中的 Joint 部分；

- •
  
  整个机器人模型构成了一棵 ​**​树形结构（Kinematic Tree）​**​，其中 base_link 通常是树的根节点，其它连杆通过关节连接形成分支。

KDL 运动学树的主要用途包括：

- •
  
  正向运动学（Forward Kinematics）：根据关节角度计算末端位姿；

- •
  
  逆向运动学（Inverse Kinematics，有限支持）；

- •
  
  雅可比矩阵计算（用于力控、速度控制等）；

- •
  
  动力学相关计算（如质量矩阵、重力补偿等，属于 KDL Dynamics 部分）

## 使用 URDF 文件配合 robot_state_publisher 工作

```cpp
   1 #include <string>
   2 #include <ros/ros.h>
   3 #include <sensor_msgs/JointState.h>
   4 #include <tf/transform_broadcaster.h>
   5 
   6 int main(int argc, char** argv) {
   7     ros::init(argc, argv, "state_publisher");
   8     ros::NodeHandle n;
   9     ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  10     tf::TransformBroadcaster broadcaster;
  11     ros::Rate loop_rate(30);
  12 
  13     const double degree = M_PI/180;
  14 
  15     // robot state
  16     double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
  17 
  18     // message declarations
  19     geometry_msgs::TransformStamped odom_trans;
  20     sensor_msgs::JointState joint_state;
  21     odom_trans.header.frame_id = "odom";
  22     odom_trans.child_frame_id = "axis";
  23 
  24     while (ros::ok()) {
  25         //update joint_state
  26         joint_state.header.stamp = ros::Time::now();
  27         joint_state.name.resize(3);
  28         joint_state.position.resize(3);
  29         joint_state.name[0] ="swivel";
  30         joint_state.position[0] = swivel;
  31         joint_state.name[1] ="tilt";
  32         joint_state.position[1] = tilt;
  33         joint_state.name[2] ="periscope";
  34         joint_state.position[2] = height;
  35 
  36 
  37         // update transform
  38         // (moving in a circle with radius=2)
  39         odom_trans.header.stamp = ros::Time::now();
  40         odom_trans.transform.translation.x = cos(angle)*2;
  41         odom_trans.transform.translation.y = sin(angle)*2;
  42         odom_trans.transform.translation.z = .7;
  43         odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
  44 
  45         //send the joint state and transform
  46         joint_pub.publish(joint_state);
  47         broadcaster.sendTransform(odom_trans);
  48 
  49         // Create new robot state
  50         tilt += tinc;
  51         if (tilt<-.5 || tilt>0) tinc *= -1;
  52         height += hinc;
  53         if (height>.2 || height<0) hinc *= -1;
  54         swivel += degree;
  55         angle += degree/4;
  56 
  57         // This will adjust as needed per iteration
  58         loop_rate.sleep();
  59     }
  60 
  61 
  62     return 0;
  63 }
```

# Using Xacro to Clean Up a URDF File

顾名思义，xacro 是一种面向 XML 的宏语言。xacro 程序会执行所有宏定义并输出最终结果

xacro --inorder model.xacro > model.urdf

在 ROS Melodic 及更高版本发行版中，你应该省略 `{--inorder}`参数。

你也可以在启动文件（launch
 file）中自动生成 URDF 文件。这种方式很方便，因为生成的 URDF 
始终是最新的，而且不会占用硬盘空间。不过，由于需要实时生成，启动文件可能需要更长的加载时间（比如 pr2_description 
就是个典型例子）。

```xml
   1 <param name="robot_description"
   2   command="xacro --inorder '$(find pr2_description)/robots/pr2.urdf.xacro'" />
```

在 URDF 文件的顶部，你必须指定一个命名空间（namespace），这样文件才能正确解析。例如，下面是一个有效的 xacro 文件的前两行：

```xml
   1 <?xml version="1.0"?>
   2 <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="firefighter">
```

这里的信息有些冗余——我们两次指定了圆柱体的长度和半径。更麻烦的是，如果想要修改这些参数，就必须在两个不同的地方分别进行更改。

幸运的是，xacro 允许你将属性定义为常量。我们可以用下面的写法来替代上面的代码。

```xml
   1   <link name="base_link">
   2     <visual>
   3       <geometry>
   4         <cylinder length="0.6" radius="0.2"/>
   5       </geometry>
   6       <material name="blue"/>
   7     </visual>
   8     <collision>
   9       <geometry>
  10         <cylinder length="0.6" radius="0.2"/>
  11       </geometry>
  12     </collision>
  13   </link>


   1 <xacro:property name="width" value="0.2" />
   2 <xacro:property name="bodylen" value="0.6" />
   3 <link name="base_link">
   4     <visual>
   5         <geometry>
   6             <cylinder radius="${width}" length="${bodylen}"/>
   7         </geometry>
   8         <material name="blue"/>
   9     </visual>
  10     <collision>
  11         <geometry>
  12             <cylinder radius="${width}" length="${bodylen}"/>
  13         </geometry>
  14     </collision>
  15 </link>
```
