# camera

## 示例

```cpp
//获得相机参数
void camerainfo::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    _frame_id = msg->header.frame_id;
    _height = msg->height;
    _width = msg->width;
    _D = msg->D;
    _K = msg->K;
    _R = msg->R;
    _P = msg->P;
    _cam_model_initialized = true;
    std::cout<<"_cam_model_initialized = true;"<<std::endl;
    _sub.shutdown();//关闭订阅
}
```

### **1. D（Distortion Coefficients，畸变系数）​**​

- ​**​含义​**​：描述相机镜头的光学畸变参数，用于修正图像的畸变（如桶形畸变、枕形畸变）。
- ​**​数据内容​**​：通常为一个一维数组，存储畸变模型的参数。ROS默认使用​**​plumb_bob畸变模型​**​（适用于普通广角/长焦镜头），包含5个参数：  
  `[k1, k2, p1, p2, k3]`
  - `k1, k2`：一阶和二阶径向畸变系数（控制径向畸变的强度）。
  - `p1, p2`：一阶和二阶切向畸变系数（控制因镜头安装倾斜导致的切向畸变）。
  - `k3`：三阶径向畸变系数（用于高阶畸变修正，常见于鱼眼镜头或大视场镜头）。

### ​**​2. K（Camera Matrix，相机内参矩阵）​**​

- ​**​含义​**​：描述相机将三维世界坐标投影到二维图像平面的几何关系，仅与相机本身的物理结构有关（与外部场景无关）。

- ​**​数据内容​**​：一个3×3的矩阵，结构为：
  
  K=$ \begin{bmatrix} f_{x} & 0 & c_{x} \\ 0 & f_{y} & c_{y} \\ 0 & 0 & 1\end{bmatrix} $

- ​
  
  - `f_x, f_y`：焦距（单位：像素），表示镜头中心到图像平面的距离（像素单位）。
  - `c_x, c_y`：主点（Principal Point）坐标（单位：像素），即光轴与图像平面的交点，通常接近图像中心（但因制造误差可能略有偏移）。

### ​**​3. R（Rectification Matrix，矫正矩阵）​**​

- ​**​含义​**​：用于立体视觉中消除左右相机图像的视差，将原始图像转换为“矫正后”的图像（对应点在同一行）。
- ​**​数据内容​**​：一个3×3的旋转矩阵（正交矩阵，行列式为1）。
  - 对于单目相机，`R`通常是3×3的单位矩阵（无需矫正）。
  - 对于立体相机中的每个相机（如左/右相机），`R`表示该相机相对于参考相机（通常为左相机）的旋转矩阵，用于对齐两个相机的坐标系。

### ​**​4. P（Projection Matrix，投影矩阵）​**​

- ​**​含义​**​：描述从三维世界坐标（齐次坐标）到二维图像像素坐标的投影关系，结合了内参矩阵和外部参数（旋转+平移）。

- ​**​数据内容​**​：一个3×4的矩阵，结构可分解为 `P = K × [R | t]`（其中`[R | t]`是3×4的外参矩阵，包含旋转`R`和平移`t`）。具体形式为：
  
  P=$ \begin{bmatrix} f_{x} & 0 & c_{x} & -f_{x}*t_{x}/t_{z}\\ 0 & f_{y} & c_{y} & -f_{y}*t_{y}/t_{z}\\ 0 & 0 & 1 & 0 \end{bmatrix} $

- ​
  
  - 前3列是内参矩阵`K`（去掉最后一行的1）。
  - 第4列是外参的投影部分（由平移向量`t`和焦距`f_x, f_y`计算得到）。

### ​**​总结​**​

这些参数共同构成了相机的完整成像模型，是相机标定后的核心输出。在ROS中，`cameraInfoCallback`函数通过接收`CameraInfo`消息获取这些参数，初始化相机模型后关闭订阅（因标定信息通常只需获取一次）。后续任务（如三维重建、目标定位）可基于这些参数将图像像素坐标转换为三维世界坐标，或反之。

## 相机标定

```bash
rosrun camera_calibration cameracalibrator.py \
  --size 9x6 \          # 角点行列数（根据标定板实际调整）
  --square 0.025 \      # 单个格子边长（米，根据标定板实际调整）
  --topic /usb_cam/image_raw \  # 替换为你的图像话题
  --camera /usb_cam \   # 替换为你的相机命名空间
  --model pinhole       # 相机模型（pinhole 普通相机；fisheye 鱼眼）
```

- `--size`：标定板的角点数量（如 `9x6`）。

- `--square`：单个格子的实际尺寸（米），​**​必须与标定板一致​**​（否则标定参数错误）。

- `--topic`：相机发布的图像话题（通过 `rostopic list`查看）。

- `--camera`：相机的命名空间（若话题为 `/usb_cam/image_raw`，则命名空间通常是 `/usb_cam`）。

- `--model`：相机模型（普通相机用 `pinhole`，鱼眼用 `fisheye`）。

```bash
#单相机
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \          # 棋盘格内部角点数（宽x高）
    --square 0.024 \      # 每个方格边长（米）
    image:=/camera/image_raw \
    camera:=/camera

#双相机
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \
    --square 0.024 \
    --approximate 0.1 \    # 图像同步时间容差（秒）
    right:=/right_camera/image_raw \
    left:=/left_camera/image_raw \
    left_camera:=/left_camera \
    right_camera:=/right_camera
```

### 如果你知道相机的设备文件名（如 `/dev/video2`），建议直接用：

```cpp
cv::VideoCapture cap("/dev/video2");
```

```bash
v4l2-ctl -d /dev/video0 --all#相机详细信息
```
