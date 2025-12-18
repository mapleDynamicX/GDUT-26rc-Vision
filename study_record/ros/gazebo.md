# Gazebo

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="paused" value="false"/>
  <arg name="use_sim_time" value="true"/>
  <arg name="gui" value="true"/>
  <arg name="recording" value="false"/>
  <arg name="debug" value="false"/>
</include>
```

> - 启动Gazebo仿真环境，使用`empty_world`模板。参数含义：
>   - `paused=false`：启动后不暂停仿真。
>   - `use_sim_time=true`：使用Gazebo的仿真时间。
>   - `gui=true`：启用Gazebo图形界面。
>   - `recording=false`：禁用场景录制。
>   - `debug=false`：禁用调试输出。

```xml
<link name="world"/> <!-- 虚拟的世界坐标系 -->
<joint name="world_joint" type="fixed">
  <parent link="world"/>
  <child link="base_footprint"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

</div>

> - 机器人有固定的安装基座（如工业机械臂）
> - 移动机器人的初始位置参考点
> - 需要固定参考坐标系时

```xml
<joint name="world_joint" type="floating">
  <parent link="world"/>
  <child link="base_link"/>
</joint>
```

> - 无固定基座的自由移动物体（如无人机、空间机器人）
> - 需要在仿真中自由拖动的物体
> - SLAM 建图时的初始无约束状态

## 要在 Gazebo 仿真中实时获取并发布机器人的位置，有几种常用方法：

### 1. 使用 Gazebo 提供的模型状态插件（推荐）

在你的 URDF 文件中添加以下插件：

```xml
<gazebo>
 <plugin name="gazebo_ros_link_state" filename="libgazebo_ros_p3d.so">
 <bodyName>base_link</bodyName>
 <topicName>/robot_pose</topicName>
 <updateRate>100.0</updateRate>
 <gaussianNoise>0.001</gaussianNoise>
 <frameName>world</frameName>
 </plugin>
</gazebo>
```
