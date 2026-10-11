#include <string>

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "LIVMapper.h"
#include "livo_atlas/ros/camera_config.h"
#include "livo_atlas/ros/sensor_extrinsics.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  livo_atlas::SensorExtrinsics sensor_extrinsics;
  std::string error;
  if (!livo_atlas::LoadSensorExtrinsics(nh, &sensor_extrinsics, &error)) {
    ROS_ERROR_STREAM("Invalid sensor extrinsics: " << error);
    return 1;
  }

  livo_atlas::CameraConfig camera_config;
  if (!livo_atlas::ros::LoadCameraConfig(
          nh, "laserMapping", &camera_config, &error)) {
    ROS_ERROR_STREAM("Invalid camera config: " << error);
    return 1;
  }

  image_transport::ImageTransport it(nh);
  LIVMapper mapper(sensor_extrinsics, camera_config, nh);
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}
