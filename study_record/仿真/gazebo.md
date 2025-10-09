# Gazebo

## URDF文件和Gazebo颜色显示原理

#### 使用Gazebo预定义材质名称

```xml
<material name="green">
  <color rgba="0 1 0 1"/>  <!-- 保留颜色定义备用 -->
</material>

<material name="green">
  <script>Gazebo/Green</script>  <!-- 使用Gazebo材质脚本 -->
</material>
```

#### 通过`<gazebo>`标签强制覆盖

```xml
<gazebo reference="base_link">
  <material>Gazebo/Green</material>
</gazebo>
```

### 需在URDF中通过`mesh`标签显式绑定

```xml
<geometry>
  <mesh filename="package://area/meshes/base_link.STL" color="green"/>
</geometry>
```

## 完整示例

## 一、加载场地 URDF 到参数服务器

```xml
<param name="arena_description" 
       command="$(find xacro)/xacro '$(find zwei)/urdf/zwei.urdf'"/>
```

> ### 说明：
> 
> - •
>   
>   ​**​目的​**​：通过 `xacro`工具处理一个 URDF 文件（可能是经过 xacro 宏处理的），并将处理后的 ​**​URDF 文本内容存储到 ROS 参数服务器​**​ 上，参数名为 `arena_description`。
> 
> - •
>   
>   ​**​参数解释​**​：
>   
>   - •
>     
>     `name="arena_description"`：这是存储在参数服务器上的参数名称，后续可以通过这个名称访问该 URDF 内容。
>   
>   - •
>     
>     `command="$(find xacro)/xacro ..."`：调用 `xacro`工具来解析 URDF/xacro 文件。
>     
>     - •
>       
>       `$(find xacro)`：这是 ROS 的包查找语法，找到名为 `xacro`的包的路径，通常用于调用 `xacro`可执行文件。
>     
>     - •
>       
>       `$(find zwei)/urdf/zwei.urdf`：找到名为 `zwei`的 ROS 包下的 `urdf/zwei.urdf`文件路径。
> 
> - •
>   
>   ​**​用途​**​：这个 URDF 很可能描述的是一个 ​**​比赛场地/竞技场/仿真环境结构​**​，之后会通过 spawn_model 将其加载进 Gazebo。

## 二、启动 Gazebo 仿真环境

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="world_name" value="$(find zwei)/worlds/competition.world"/>
  <arg name="paused" value="false"/>
  <arg name="use_sim_time" value="true"/>
  <arg name="gui" value="true"/>
  <arg name="headless" value="false"/>
  <arg name="debug" value="false"/>
</include>
```

> ### 说明：
> 
> - •
>   
>   ​**​目的​**​：通过 `include`引入 Gazebo 提供的一个标准 launch 文件 `empty_world.launch`，即启动一个​**​空的 Gazebo 仿真世界​**​，但同时可以指定自定义的世界文件。
> 
> - •
>   
>   ​**​包含的文件​**​：`$(find gazebo_ros)/launch/empty_world.launch`是 ROS 中 gazebo_ros 功能包提供的一个常用 launch 文件，用于快速启动 Gazebo。
> 
> - •
>   
>   ​**​传递的参数（args）​**​：
>   
>   - •
>     
>     `world_name="$(find zwei)/worlds/competition.world"`：​**​指定要加载的 Gazebo 世界文件​**​，这里是 `zwei`包下的 `worlds/competition.world`。如果指定了该参数，Gazebo 将加载这个自定义世界，而不是默认的空世界。
>     
>     - •
>       
>       若该参数未设置或为空，则 Gazebo 启动默认的空世界。
> 
> - •
>   
>   `paused="false"`：Gazebo 启动时​**​不暂停仿真​**​，机器人和物理引擎立即开始运行。
> 
> - •
>   
>   `use_sim_time="true"`：告诉 ROS 系统使用 Gazebo 提供的仿真时间（而不是系统真实时间）。这对于同步 ROS 节点与仿真时间非常重要。
> 
> - •
>   
>   `gui="true"`：​**​显示 Gazebo 的图形化界面（GUI）​**​，即你可以看到仿真的 3D 窗口。
> 
> - •
>   
>   `headless="false"`：不启用无头模式（即不隐藏 GUI，与 gui=true 一致）。
> 
> - •
>   
>   `debug="false"`：不启用调试模式。
> 
> ### 总结：
> 
> 此部分启动了 Gazebo 仿真环境，并加载了一个自定义的仿真世界文件 `competition.world`（可能包含地面、障碍物或其他环境元素）。

## 三、在 Gazebo 中生成场地模型

```xml
<node name="spawn_arena" pkg="gazebo_ros" type="spawn_model"
      args="-urdf -param arena_description -model competition_area -x 0 -y 0 -z 0"
      respawn="false" output="screen"/>
```

> ### 说明：
> 
> - •
>   
>   ​**​目的​**​：使用 `gazebo_ros`提供的 `spawn_model`工具，将一个之前存放在 ROS 参数中的 URDF 模型（即 `arena_description`，也就是场地模型）​**​加载进 Gazebo 仿真环境中​**​。
> 
> - •
>   
>   ​**​节点信息​**​：
>   
>   - •
>     
>     `name="spawn_arena"`：该节点的名称。
>   
>   - •
>     
>     `pkg="gazebo_ros"`：节点来自 `gazebo_ros`包。
>   
>   - •
>     
>     `type="spawn_model"`：调用的是 `spawn_model`工具，它可以将模型（URDF/SDF）加载进 Gazebo。
> 
> - •
>   
>   ​**​关键参数（args）​**​：
>   
>   - •
>     
>     `-urdf`：表示要加载的模型是 ​**​URDF 格式​**​。
>   
>   - •
>     
>     `-param arena_description`：表示 URDF 的内容来自 ROS 参数服务器上名为 `arena_description`的参数（就是前面定义的那个）。
>   
>   - •
>     
>     `-model competition_area`：在 Gazebo 中，这个模型会以名称 `competition_area`注册，你可以用这个名称在 Gazebo 中查找或控制它。
>   
>   - •
>     
>     `-x 0 -y 0 -z 0`：设置模型在 Gazebo 世界中的初始位置（此处为原点）。
> 
> - •
>   
>   `respawn="false"`：如果节点退出，不会自动重启。
> 
> - •
>   
>   `output="screen"`：将节点的输出打印到终端屏幕，方便调试。
> 
> ### 总结：
> 
> 这一部分将前面通过 xacro 处理并存储在参数中的场地 URDF 模型（可能是竞技场或环境结构）​**​加载进 Gazebo 中​**​，作为静态环境存在。

## 五、在 Gazebo 中加载真正的机器人模型

```xml
<node name="spawn_robot" pkg="gazebo_ros" type="spawn_model" 
      args="-urdf -param robot_description -model mbot -x 1 -y 1 -z 0"
      respawn="false" output="screen"/>
```

> ### 说明：
> 
> - •
>   
>   ​**​目的​**​：将机器人模型（通过 `robot_description`参数指定的 URDF）加载进 Gazebo，模型名为 `mbot`，并设置其在世界中的初始位置为 `(x=1, y=1, z=0)`，避免与场地发生碰撞。
> 
> - •
>   
>   ​**​关键参数​**​：
>   
>   - •
>     
>     `-urdf`：加载 URDF 格式的模型。
>   
>   - •
>     
>     `-param robot_description`：机器人模型的 URDF 内容来自 ROS 参数服务器的 `robot_description`参数。
>   
>   - •
>     
>     `-model mbot`：在 Gazebo 中该机器人模型的名称为 `mbot`，用于标识。
>   
>   - •
>     
>     `-x 1 -y 1 -z 0`：设置机器人在 Gazebo 世界中的初始位置，避免与场地中心（0,0,0）重叠或碰撞。
> 
> - •
>   
>   ​**​作用​**​：将机器人放入 Gazebo 仿真世界中，准备进行控制、仿真、导航等任务。

## 六、定义机器人模型文件路径 & 加载到参数服务器

```xml
<arg name="model" default="/home/maple/study2/world_ws4/src/wpr_simulation/models/wpb_home.model"/>
<param name="robot_description" command="$(find xacro)/xacro $(arg model)" />
```

> ### 说明：
> 
> - •
>   
>   ​**​目的​**​：定义机器人模型的文件路径，并通过 `xacro`工具将其加载到 ROS 参数服务器的 `robot_description`参数中，供后续 spawn_model 使用。
> 
> - •
>   
>   ​**​参数解释​**​：
>   
>   - •
>     
>     `<arg name="model" default="...">`：定义一个 launch 文件内部的参数 `model`，代表机器人模型的文件路径，默认是 `/home/maple/study2/world_ws4/src/wpr_simulation/models/wpb_home.model`。
>     
>     - •
>       
>       这个路径看起来是一个 ​**​.model 文件​**​，但注意：​**​通常机器人模型是 URDF 或 xacro 文件，不是 .model 文件​**​。请确认该文件实际格式！
>     
>     - •
>       
>       如果它是 ​**​xacro 文件​**​，那用 xacro 解析是正确的；如果是 ​**​SDF 或其他格式​**​，可能需要调整。
> 
> - •
>   
>   `<param name="robot_description" ...>`：调用 `xacro`工具处理该模型文件，并将结果（应该是 URDF 格式的文本）存入 ROS 参数服务器的 `robot_description`参数中。
>   
>   - •
>     
>     这是 ROS 中​**​标准做法​**​，几乎所有机器人控制、导航、RViz 可视化等都依赖 `robot_description`参数。

## 七、发布机器人状态信息

```xml
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
```

> ### 说明：
> 
> 这两行是用于 ​**​发布机器人状态信息​**​，使机器人模型能够在 RViz 中正确显示，并让其他节点获取机器人的 TF（坐标变换）和关节状态。
> 
> ### 1. joint_state_publisher
> 
> - •
>   
>   ​**​作用​**​：订阅机器人的关节状态信息（比如关节角度），如果没有其他节点发布，它还可以提供一个简单的 GUI 让用户手动调节关节值（主要用于仿真或原型阶段）。
> 
> - •
>   
>   ​**​pkg="joint_state_publisher"​**​：ROS 提供的标准功能包。
> 
> - •
>   
>   ​**​type="joint_state_publisher"​**​：对应的节点可执行程序。
