#include "LIVMapper.h"

#include <exception>

int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  try {
    LIVMapper mapper(nh);
    mapper.initializeSubscribersAndPublishers(nh, it);
    mapper.run();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("LIVO initialization or processing failed: "
                     << error.what());
    return 1;
  }
  return 0;
}
