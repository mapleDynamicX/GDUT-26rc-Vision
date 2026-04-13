# 录制bag

## 2️⃣ 查看可录制的话题

```bash
rostopic list
rostopic info /topic_name  # 查看话题类型
```

## 3️⃣ 基本录制命令

**录制单个话题：**

```bash
rosbag record /topic_name
```

**录制多个话题：**

```bash
rosbag record /topic1 /topic2 /topic3
```

**录制所有话题：**

```bash
rosbag record -a
```

**指定输出文件名：**

```bash
rosbag record -O my_data.bag /topic_name
```

## 4️⃣ 查看录制内容

```bash
rosbag info my_data.bag
```

## 5️⃣ 回放数据

```bash
rosbag play my_data.bag
```

## 6️⃣ 常见问题

- **权限问题**：确保对当前目录有写权限
- **话题未发布**：录制前确认话题有数据
- **时间同步**：确保系统时间正确
- **文件过大**：可使用 `-b` 参数设置缓冲区

## 7️⃣ 实用技巧

- **按时间录制**：`rosbag record -a -d 300` (录制 5 分钟)
- **压缩数据**：`rosbag record -a -j` (使用 bz2 压缩)
- **过滤话题**：`rosbag record -a --exclude /tf`

需要更详细的示例或者特定场景下的录制方案吗？我可以提供更具体的命令。

### 播放时加上时间戳

```bash
rosbag play my_data.bag --clock
```

### 指定播放倍速（控制整体频率）

```bash
# 基础格式
rosbag play -r 倍率 你的包.bag

# 常用示例
rosbag play -r 1.0 test.bag   # 原始速度（默认）
rosbag play -r 0.5 test.bag   # 0.5倍速（减速，频率减半）
rosbag play -r 2 test.bag     # 2倍速（加速，频率翻倍）
rosbag play -r 10 test.bag    # 10倍速快速回放

```
