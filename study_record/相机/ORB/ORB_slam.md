# ORB_slam

and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

```bash
export ROS_PACKAGE_PATH=/home/maple/study2/github/ORB_SLAM3/Examples/ROS
```

```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/maple/study2/ORB_SLAM3/Examples/ROS
```

## 运行

**intel**

```bash
rosrun ORB_SLAM3 Mono /home/maple/study2/github/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/maple/study2/github/ORB_SLAM3/Examples/ROS/ORB_SLAM3/intel.yaml
```

```bash
rosrun ORB_SLAM3 Mono /home/maple/study2/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/maple/study2/ORB_SLAM3/Examples/ROS/ORB_SLAM3/intel.yaml
```

**usb**

```bash
rosrun ORB_SLAM3 Mono /home/maple/study2/github/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/maple/study2/github/ORB_SLAM3/Examples/ROS/ORB_SLAM3/Asus.yaml
```

```bash
rosrun ORB_SLAM3 Mono /home/maple/study2/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/maple/study2/ORB_SLAM3/Examples/ROS/ORB_SLAM3/Asus.yaml
```
