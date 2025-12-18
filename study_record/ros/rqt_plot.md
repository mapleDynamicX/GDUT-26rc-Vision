# rqt_plot

```bash
sudo apt-get install ros-noetic-rqt-plot
```

#### 注意事项

- `rqt_plot` 仅支持**数值型字段**（int/float），不支持字符串、几何点云等非数值类型；
- 话题路径需精确到字段（如`/temp/data`，而非仅`/temp`）；
- 可通过 GUI 界面（左上角 “Add” 按钮）手动添加 / 删除话题，调整绘图范围、刷新率。

要绘制 nav_msgs/Odometry消息，核心是**提取该消息中的数值型字段**（如位姿、线速度、角速度），再通过 `rqt_plot`（快速）或自定义 Python 代码（灵活定制）可视化。

`nav_msgs/Odometry` 是 ROS 中描述机器人位姿和速度的核心消息，其关键数值字段层级如下：

| 类型      | 字段路径（需精确指定）                     | 含义            |
| ------- | ------------------------------- | ------------- |
| 位置      | `pose/pose/position/x/y/z`      | 机器人在坐标系中的三维位置 |
| 姿态（四元数） | `pose/pose/orientation/x/y/z/w` | 机器人姿态（四元数）    |
| 线速度     | `twist/twist/linear/x/y/z`      | 机器人线速度（m/s）   |
| 角速度     | `twist/twist/angular/x/y/z`     | 机器人角速度（rad/s） |

### 1. 基础用法

核心命令格式：

```bash
rqt_plot [Odometry话题名]/[字段路径]
```

#### 步骤 2：启动 rqt_plot 绘制目标字段

##### 示例 1：绘制海龟的 x/y 位置（轨迹的时间序列）

```bash
rqt_plot /turtle1/odom/pose/pose/position/x /turtle1/odom/pose/pose/position/y
```

窗口会显示 x、y 位置随时间变化的曲线（能看到海龟移动的位置波动）。

```
rqt_bag xx.bag
```
