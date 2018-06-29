
#include <ros/ros.h>
#include "point_cloud_io/Read.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "read");
  ros::NodeHandle nodeHandle("~");

  point_cloud_io::Read read(nodeHandle);

  ros::spin();
  return 0;
}
