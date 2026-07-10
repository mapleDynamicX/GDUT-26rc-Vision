#include "LIVMapper.h"





int main(int argc, char **argv)
{
  ros::init(argc, argv, "fastlivo_mapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // 全程只创建一个对象，订阅只初始化一次
  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh, it);

  ros::Rate sl(10);
  while(ros::ok())
  {
    mapper.run();
    // 检测到重启后，重置内部状态，直接重新进入run
    mapper.reset();
    sl.sleep();
  }
  return 0;
}
