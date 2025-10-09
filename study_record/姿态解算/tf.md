## tf

## 发布tf2_msgs/TFMessage

```cpp
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "manual_tf_publisher");
    ros::NodeHandle nh;

    // 创建一个 publisher，发布到 "/tf" topic，消息类型为 tf2_msgs::TFMessage
    ros::Publisher tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 10);

    ros::Rate rate(10.0);

    while (ros::ok()){
        tf2_msgs::TFMessage tf_msg;

        // 构造一个 TransformStamped
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = 1.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        // 将单个 TransformStamped 放入 TFMessage 中
        tf_msg.transforms.push_back(transformStamped);

        // 发布 TFMessage
        tf_pub.publish(tf_msg);

        rate.sleep();
    }

    return 0;
}
```

other

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_publisher_example");
    ros::NodeHandle node;

    // 创建一个 TF 广播器
    static tf2_ros::TransformBroadcaster br;

    ros::Rate rate(10.0);
    while (ros::ok()){
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = 1.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        br.sendTransform(transformStamped);

        rate.sleep();
    }

    return 0;
};
```

## 订阅tf

```cpp
// 回调函数：处理接收到的 TF 消息
void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    for (const auto& transform : msg->transforms)
    {
        if(transform.child_frame_id != "body")
        {
            continue;
        }
        // 获取四元数
        geometry_msgs::Quaternion quat = transform.transform.rotation;

        // 使用 tf2 库将四元数转换为欧拉角
        tf2::Quaternion tf_quat;
        tf_quat.setX(quat.x);
        tf_quat.setY(quat.y);
        tf_quat.setZ(quat.z);
        tf_quat.setW(quat.w);

        // 转换为旋转矩阵，然后提取欧拉角
        tf2::Matrix3x3 mat(tf_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);  // 得到的是弧度值

        ROS_INFO_STREAM("Transform from " << transform.header.frame_id 
                      << " to " << transform.child_frame_id 
                      << "\nRoll: " << roll 
                      << " [rad], Pitch: " << pitch 
                      << " [rad], Yaw: " << yaw 
                      << " [rad]");
        float msgs[3] = {0};
        msgs[0] = transform.transform.translation.x;
        msgs[1] = transform.transform.translation.y;
        msgs[2] = yaw;
        serialComm->serial_send(0, msgs, 3);
    }
}
```
