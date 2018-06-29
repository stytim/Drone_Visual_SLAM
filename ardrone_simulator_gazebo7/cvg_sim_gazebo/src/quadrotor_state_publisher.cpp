#include "string"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

ros::Publisher  * joint_publisher_ptr;
ros::Publisher  * odometry_publisher_ptr;

void EduMipState_Callback(const edumip_msgs::EduMipState::ConstPtr& EduMipState)
{

  static tf::TransformBroadcaster tf_broadcaster;
  static tf::Transform world_to_edumip;
  static sensor_msgs::JointState joint_state;
  world_to_edumip.setOrigin( tf::Vector3( EduMipState->body_frame_northing,
						-EduMipState->body_frame_easting,
					        0.034));
  tf::Quaternion q;
  q.setRPY(0.0 , EduMipState->theta, -EduMipState->body_frame_heading);
  world_to_edumip.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(world_to_edumip,  ros::Time::now(), "world", "edumip_body"));
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] ="jointL";
  joint_state.name[1] ="jointR";
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = EduMipState->wheel_angle_L;
  joint_state.position[1] = EduMipState->wheel_angle_R;
  joint_publisher_ptr->publish(joint_state);

  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "world";
  odometry.child_frame_id = "edumip_body";
  tf::poseTFToMsg(world_to_edumip, odometry.pose.pose);
  odometry_publisher_ptr->publish(odometry);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edumip_my_robot_state_publisher");
  ros::NodeHandle n;
  ros::Publisher  joint_publisher  = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  ros::Publisher  odom_publisher  = n.advertise<nav_msgs::Odometry>("/edumip/odometry", 10);
  joint_publisher_ptr = &joint_publisher;
  odometry_publisher_ptr = &odom_publisher;
  ros::Subscriber sub = n.subscribe("edumip/state", 100, EduMipState_Callback);
  ros::spin();
  return 0;
}
