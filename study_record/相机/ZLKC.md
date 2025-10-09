# ZLKC

```xml
<!-- =================================================================== -->
<!-- JHCap 相机启动文件（Launch File）                                 -->
<!-- 用于启动 JHCap 相机驱动节点，并配置相机参数、发布图像话题          -->
<!-- =================================================================== -->
<launch>

  <!-- ============================================== -->
  <!-- 相机节点参数定义（arg）                         -->
  <!-- arg 用于声明可外部传入的参数，方便启动时覆盖默认值 -->
  <!-- ============================================== -->

  <!-- 相机名称，用于 ROS 话题命名空间（默认 jhcap_camera） -->
  <!-- 例如话题会变成 /jhcap_camera/image_raw             -->
  <arg name="camera_name" default="jhcap_camera" />

  <!-- 相机设备 ID，通常是 /dev/videoX 的编号             -->
  <!-- 多相机时可改为 1、2 等，对应不同设备               -->
  <arg name="camera_id" default="0" />

  <!-- 图像帧 ID（TF 坐标系名称），用于与其他传感器进行坐标变换 -->
  <arg name="frame_id" default="camera" />

  <!-- 输出图像宽度（像素），需相机硬件支持                 -->
  <arg name="image_width" default="1920" />

  <!-- 输出图像高度（像素），需相机硬件支持                -->
  <arg name="image_height" default="1080" />

  <!-- 帧率（FPS），需相机硬件支持，过高可能掉帧           -->
  <arg name="frame_rate" default="55.0" />

  <!-- 增益（Gain），数值越大图像越亮，但噪点可能增多     -->
  <!-- 单位和范围由相机驱动决定，这里 174 是示例值         -->
  <arg name="gain" default="174" />

  <!-- 曝光时间（Exposure），单位通常是微秒（μs）          -->
  <!-- 数值越大画面越亮，但运动模糊风险增加               -->
  <arg name="exposure" default="1000" />

  <!-- 是否开启自动曝光（true/false）                    -->
  <arg name="auto_exposure" default="false" />

  <!-- 是否开启自动增益（true/false）                      -->
  <arg name="auto_gain" default="false" />

  <!-- JPEG 压缩质量（1-100），100 表示无损压缩           -->
  <!-- 仅在发布压缩图像话题时有效                          -->
  <arg name="jpeg_quality" default="100" />

  <!-- 相机标定信息文件路径（YAML 格式）                  -->
  <!-- 用于发布 camera_info 话题，包含内参矩阵、畸变系数等 -->
  <arg name="camera_info_url" default="package://jhcap_camera/config/camera_info.yaml" />

  <!-- 是否开启调试模式（true 时可启动额外的可视化节点）  -->
  <arg name="debug" default="false" />

  <!-- 启动前缀，用于调试（例如 gdb --ex run --args 或 valgrind） -->
  <arg name="launch_prefix" default="" />

  <!-- 节点崩溃后是否自动重启（true/false）               -->
  <arg name="respawn" default="false" />

  <!-- ============================================== -->
  <!-- 节点命名空间（Group Namespace）                 -->
  <!-- 将下面的节点和话题放入以 camera_name 为名称的命名空间 -->
  <!-- 避免多相机时话题冲突                            -->
  <!-- ============================================== -->
  <group ns="$(arg camera_name)">

    <!-- ============================================== -->
    <!-- 启动 JHCap 相机驱动节点                         -->
    <!-- pkg: 功能包名称                                 -->
    <!-- type: 可执行文件名称                            -->
    <!-- name: 节点运行时名称                            -->
    <!-- output: 日志输出位置（screen 表示打印到终端）    -->
    <!-- launch-prefix: 启动前缀，用于调试                -->
    <!-- respawn: 是否自动重启                            -->
    <!-- ============================================== -->
    <node name="jhcap_camera" 
          pkg="jhcap_camera" 
          type="jhcap_camera" 
          output="screen"
          launch-prefix="$(arg launch_prefix)"
          respawn="$(arg respawn)">

      <!-- 将前面声明的 arg 参数作为 ROS 参数服务器中的参数传递给节点 -->
      <param name="camera_name" value="$(arg camera_name)" />
      <param name="camera_id" value="$(arg camera_id)" />
      <param name="frame_id" value="$(arg frame_id)" />
      <param name="image_width" value="$(arg image_width)" />
      <param name="image_height" value="$(arg image_height)" />
      <param name="frame_rate" value="$(arg frame_rate)" />
      <param name="gain" value="$(arg gain)" />
      <param name="exposure" value="$(arg exposure)" />
      <param name="auto_exposure" value="$(arg auto_exposure)" />
      <param name="auto_gain" value="$(arg auto_gain)" />
      <param name="jpeg_quality" value="$(arg jpeg_quality)" />

      <!-- 相机标定信息文件路径参数 -->
      <param name="camera_info_url" value="$(arg camera_info_url)" />

      <!-- 话题重映射（remap）：将节点内部发布的话题名映射到新名称 -->
      <!-- 这里 from 是节点内部默认的话题名，to 是实际发布到 ROS 网络的话题名 -->
      <!-- 这里保持原名，仅作为示例展示如何重映射 -->
      <remap from="image_raw" to="image_raw" />
      <remap from="image_raw/compressed" to="image_raw/compressed" />
      <remap from="camera_info" to="camera_info" />
    </node>

    <!-- ============================================== -->
    <!-- 可选的图像处理节点（image_proc）                -->
    <!-- 提供去畸变、彩色转灰度、图像缩放等功能           -->
    <!-- 启动后会在当前命名空间下发布 rectified 等话题    -->
    <!-- ============================================== -->
    <!--
    <include file="$(find image_proc)/launch/image_proc.launch">
      <arg name="manager" value="camera_nodelet_manager" />
    </include>
    -->
  </group>

  <!-- ============================================== -->
  <!-- 可选：启动 image_view 用于实时显示图像（调试用） -->
  <!-- if="$(arg debug)" 表示只有 debug 为 true 时才启动 -->
  <!-- ============================================== -->
  <!--
  <node name="image_view" 
        pkg="image_view" 
        type="image_view" 
        output="screen"
        if="$(arg debug)">
    <remap from="image" to="/$(arg camera_name)/image_raw" />
  </node>
  -->

  <!-- ============================================== -->
  <!-- 可选：启动 rqt_image_view（ROS GUI 工具）       -->
  <!-- 更方便地选择和查看不同的图像话题                 -->
  <!-- ============================================== -->
  <!--
  <node name="rqt_image_view" 
        pkg="rqt_image_view" 
        type="rqt_image_view" 
        output="screen"
        if="$(arg debug)">
  </node>
  -->
</launch>
```

### 1. **W/T 调节环（变焦控制）**

- **W（Wide Angle）**：代表广角端，转动此环可将镜头焦距调至最短（如 6mm），用于拍摄更广阔的视野，适合大范围监控或需要覆盖较大区域的场景。
- **T（Telephoto）**：代表长焦端，转动此环可将镜头焦距调至最长（如 12mm），用于放大远处物体细节，适合需要高精度检测或特写的场景。
- **操作方式**：VM06012MP 为手动变焦设计，通过旋转镜头外侧的变焦环实现 W/T 调节，焦距变化范围为 6-12mm，支持无级调节。

### 2. **O/C 调节环（光圈控制）**

- **O（Open）**：代表光圈开大，转动此环可增加光圈值（如 F1.6），允许更多光线进入镜头，适合低光照环境或需要浅景深的场景。
- **C（Close）**：代表光圈缩小，转动此环可减小光圈值（如 F22），减少进光量，适合强光环境或需要大景深（前后景物均清晰）的场景。
- **操作方式**：光圈调节环位于镜头后部，通过手动旋转实现 O/C 调节，光圈范围为 F1.6 至 F22（具体以实际产品为准）。
