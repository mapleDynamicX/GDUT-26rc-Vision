## openvino安装

**​添加 Intel 的 GPG 公钥​**​：

```bash
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
```

**​添加 OpenVINO APT 仓库​**​

```
echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu20 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list
```

​**​更新并安装 OpenVINO​**​：

```bash
sudo apt update
sudo apt install openvino
```

#### **​驱动安装​**​

​**​Linux​**​：

安装 OpenCL 运行时和 Intel 计算库：

```bash
sudo apt install intel-opencl-icd intel-level-zero-gpu
```

验证驱动：

```bash
clinfo | grep "Device Name"  # 应显示 Intel GPU
```

#### ​**​配置 GPU 支持​**​

​**​Linux​**​：安装 GPU 插件依赖：

```bash
sudo apt install intel-gpu-tools  # 可选，用于调试
```

### ​**​5. 常见问题​**​

#### ​**​Q1：报错 `No device named 'GPU'`​**​

- ​**​原因​**​：驱动未安装或 OpenCL 运行时缺失。

​**​解决​**​：

```bash
sudo apt install intel-opencl-icd  # Linux
```
