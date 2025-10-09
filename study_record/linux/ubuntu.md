# ubuntu

## 更新签名，解决某些软件包无法安装的问题

```shell
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### **​查看整体磁盘空间（所有分区）：`df` 命令**

```bash
df
```

#### **​优化输出（易读单位，如 GB/MB）：​**​

添加 `-h` 参数（human-readable），自动转换为更易读的单位（KB/MB/GB/TB）

```bash
df -h
```

### **一、命令行工具（推荐）​**​

SSH 本身不直接传输文件，但基于 SSH 协议衍生出了多个高效工具，支持加密传输（数据安全），且操作简单。

#### ​**​`scp`（Secure Copy Protocol）—— 最基础的文件传输​**​

`scp`（Secure Copy）是最常用的 SSH 文件传输工具，适合​**​小文件或简单目录​**​的上传/下载，语法类似 `cp` 命令。

```bash
scp [选项] [源文件/目录] [目标路径]
```

##### **​常用场景示例​**​：

###### ​**​场景 1：本地文件 → 远程服务器（上传）​**​

```bash
# 格式：scp 本地文件路径 远程用户@远程IP:远程目标路径
scp /home/user/local_file.txt root@192.168.1.100:/root/remote_dir/
```

### **​一、基础删除：`rm` 命令（仅删除索引，可能恢复）​**​

`rm`（Remove）是 Linux 最常用的删除命令，但​**​默认仅删除文件系统的逻辑链接​**​，数据仍可能通过恢复工具（如 `testdisk`、`photorec`）找回。适合删除非敏感文件。

```bash
# 删除单个文件（需确认）
rm 文件名

# 强制删除（不提示，直接删）
rm -f 文件名   # -f：force（忽略不存在的文件，无提示）

# 递归删除目录（含子文件/子目录）
rm -rf 目录名   # -r：recursive（递归）；-f：强制
```

### **方法 2：通过 `lsusb` 结合 `dmesg` 定位​**​

`lsusb` 可以列出所有已识别的 USB 设备（包括厂商 ID 和产品 ID），结合 `dmesg` 可以进一步确认设备对应的串口。

#### 操作步骤：

1. ​**​查看所有 USB 设备​**​：

```bash
lsusb
```

输出示例（关键部分已标注）：

```bash
Bus 001 Device 003: ID 1a86:7523 QinHeng Electronics CH340 serial converter
```

这里 `1a86:7523` 是设备的 ​**​Vendor ID（厂商 ID）: Product ID（产品 ID）​**​，对应 CH340 芯片的串口转换器。

2. ​**​通过 `dmesg` 过滤该设备的日志​**​：

```bash
dmesg | grep -i '1a86:7523'  # 替换为你的设备的 Vendor:Product ID
```

输出示例：

```bash
[1234.567890] usb 1-2: new full-speed USB device number 3 using xhci_hcd
[1234.712345] usb 1-2: New USB device found, idVendor=1a86, idProduct=7523
[1234.712346] usb 1-2: Product: CH340 Serial Converter
[1234.712347] usb 1-2: Manufacturer: QinHeng Electronics
[1234.715678] ch341 1-2:1.0: ch341-uart converter registered
[1234.715679] usb 1-2: ch341-uart converter now attached to ttyUSB0
```

可以看到设备绑定了 `/dev/ttyUSB0`。

### **​方法 1：通过 `dmesg` 查看内核日志（最常用）​**​

当 USB 串口设备插入时，Linux 内核会在日志中记录设备的识别过程和分配的串口名称（如 `/dev/ttyUSB0`）。可以通过 `dmesg` 命令查看最近的日志。

#### 操作步骤：

1. ​**​插入 USB 串口设备​**​（如 USB 转串口线），确保设备已连接。
2. 在终端执行以下命令，过滤出与串口（`tty`）相关的日志：

```bash
dmesg | grep -iE 'tty|usb serial'
```

通过SSH远程启动另一台电脑的ROS节点后，手动关闭该电脑的WiFi，对运行程序的影响主要取决于​**​ROS节点的通信方式​**​和​**​网络接口的依赖情况​**​，具体分析如下：

### ​**​1. ROS节点的通信类型​**​

ROS节点的通信主要分为两类，影响差异较大：

- ​**​本地通信（环回接口）​**​：若节点仅在远程电脑本地运行（如通过`localhost`或`127.0.0.1`通信，例如同一台机器上的`roscore`与节点），则通信依赖​**​环回接口（loopback）​**​，与物理网络（如WiFi、有线网）无关。此时关闭WiFi​**​不会影响本地节点间的通信​**​，节点仍会正常运行。

- ​**​跨网络通信​**​：若节点需要通过网络与其他设备（如传感器、其他机器人或远程服务器）通信（例如订阅外部IP的 topic、调用远程Service），且WiFi是唯一的网络连接方式，则关闭WiFi会导致​**​外部通信中断​**​。此时节点可能因无法接收/发送数据而停止工作、报错，或进入等待状态（如订阅不到消息时卡住）。

### ​**​2. SSH连接的影响​**​

关闭WiFi会导致远程电脑的​**​网络连接中断​**​（假设WiFi是其唯一联网方式），此时：

- ​**​SSH连接会断开​**​：因为你通过WiFi建立的SSH会话依赖网络，断开后无法再通过SSH远程控制该电脑（除非有其他联网方式如有线网保持连接）。
- ​**​节点是否继续运行​**​：取决于节点的启动方式：
  - 若节点在SSH终端中​**​前台运行​**​（直接执行`rosrun`或`roslaunch`），SSH断开时终端关闭，节点会被终止（除非用`nohup`或`&`后台运行）。
  - 若节点通过`screen`、`tmux`或`systemd`等服务管理工具​**​后台持久化运行​**​，即使SSH断开，节点仍会在后台继续运行（需确认启动脚本是否正确配置）。

### ​**​3. 关键结论​**​

- ​**​仅本地通信的ROS节点​**​：关闭WiFi不影响其运行（环回接口独立），但SSH会断开，需通过其他方式（如有线网）重新连接才能控制。
- ​**​依赖外部网络的ROS节点​**​：关闭WiFi会导致外部通信中断，节点可能停止或异常（具体取决于代码逻辑）。
- ​**​节点生命周期​**​：SSH断开不直接终止节点，除非启动方式未做后台保护（如直接前台运行）。

## 安装ros2

```
sudo apt update
wget http://fishros.com/install -O fishros && bash fishros
```

## vscode安装ros1插件

**启动 vscode**

**ctrl+p**

```bash
ext install ms-iot.vscode-ros
```

#### **临时修改串口权限**

```bash
sudo chmod 666 /dev/ttyUSB0
```

#### **方法 1：将当前用户加入 `dialout`组（推荐）​**​

**​检查当前用户所属的组​**​

```bash
groups
# 或更清晰的输出：
id -nG
```

**​将用户加入 `dialout`组​**​

```bash
sudo usermod -aG dialout $USER
```

**​生效权限​**​

添加组后，需要​**​重新登录系统​**​（或重启）才能让组权限生效。

在 Ubuntu 中无法输入中文通常是由于未安装中文输入法或输入法配置不当导致的。以下是详细的解决步骤，覆盖主流的 ​**​IBus​**​（GNOME 默认）和 ​**​Fcitx​**​（可选）两种输入法框架：

```bash
sudo apt update
sudo apt install language-pack-zh-hans language-pack-zh-hans-base  # 简体中文
# 若需繁体：sudo apt install language-pack-zh-hant language-pack-zh-hant-base
```

安装后，系统会自动识别中文环境。

#### ​**​2. 安装输入法引擎​**​

根据你使用的输入法框架（IBus 或 Fcitx）选择安装对应的中文输入法引擎：

##### ​**​方案 A：使用 IBus（GNOME 桌面默认）​**​

IBus 是 GNOME 桌面的默认输入法框架，推荐新手使用。

安装 ​**​智能拼音输入法​**​（最常用）：

```bash
sudo apt install ibus ibus-libpinyin  # ibus-libpinyin 是 IBus 的拼音引擎
```

### **二、配置输入法框架​**​

安装完成后，需要将输入法添加到系统输入源中。

#### ​**​情况 1：使用 IBus（GNOME 桌面）​**​

- 打开 ​**​设置​**​ → ​**​区域与语言​**​ → 点击 ​**​管理已安装的语言​**​（或右侧 `+`号）。

- 在“添加语言”搜索框中输入“中文”，选择 ​**​中文（中国）​**​ → 点击 ​**​添加​**​。

- 等待系统自动配置，返回“区域与语言”界面，展开 ​**​输入源​**​ 选项。

- 点击 `+`号 → 搜索并添加 ​**​中文（智能拼音）​**​（对应 `ibus-libpinyin`）。

切换输入法：按 `Super（Win键）+ 空格`切换中/英文输入。

```bash
sudo journalctl --vacuum-time=2weeks  # 删除两周前的日志
```

## libusb库冲突

海康相机SDK导致

sudo rm -rf  /opt/MVS
