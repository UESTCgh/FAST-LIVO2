#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(private_nh);
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}
