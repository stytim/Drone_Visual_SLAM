
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

const int PUBLISH_FREQ = 50;

using namespace std;

struct TeleopArDrone
{
  ros::Subscriber joy_sub;
  ros::Publisher pub_takeoff, pub_land, pub_toggle_state, pub_vel, pub_enable_autopilot;

  bool got_first_joy_msg;

  bool is_flying;
  bool toggle_pressed_in_last_msg;
  bool cam_toggle_pressed_in_last_msg;
  
  bool autopilot_;
  bool autopilot_toggle_pressed_in_last_msg;
  
  std_srvs::Empty srv_empty;

	ros::NodeHandle nh_;
	geometry_msgs::Twist twist;
	ros::ServiceClient srv_cl_cam;


	void joyCb(const sensor_msgs::JoyConstPtr joy_msg){

  

	 if (!got_first_joy_msg){
    ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());

      autopilot_ = false;
	  got_first_joy_msg = true;
	 }

	 // mapping from joystick to velocity
	 float scale = 0.25;

	 twist.linear.x = scale*joy_msg->axes[1]; // forward, backward
	 twist.linear.y = scale*joy_msg->axes[0]; // left right
   	 twist.linear.z = scale*joy_msg->axes[3]; // up down
   	 twist.angular.z = scale*joy_msg->axes[2]; // yaw

	 // button 10 (L1): dead man switch
	 bool dead_man_pressed = joy_msg->buttons.at(4);
	 bool dead_man2_pressed = joy_msg->buttons.at(6);

	 // button 11 (R1): switch emergeny state 
	 bool emergency_toggle_pressed = joy_msg->buttons.at(5);

	 // button 0 (select): switch camera mode
   	 bool cam_toggle_pressed = joy_msg->buttons.at(8);

	 if(joy_msg->buttons.at(1))
		twist.angular.x = twist.angular.y = 0;
	 else
		twist.angular.x = twist.angular.y = 1;

	 if (!is_flying && dead_man_pressed){
	  ROS_INFO("LB was pressed, Taking off!");
	  pub_takeoff.publish(std_msgs::Empty());
	  is_flying = true;
	 }

	 if (is_flying && dead_man2_pressed){
	  ROS_INFO("LT was pressed, landing");
	  pub_land.publish(std_msgs::Empty());
	  is_flying = false;
	 }
	 
	 bool old_autopilot_ = autopilot_;
	 
	 // toggle autopliot with start button
	if(joy_msg->buttons.at(9))
	{
	  if(!autopilot_toggle_pressed_in_last_msg)
        autopilot_ = !autopilot_;
         
	  autopilot_toggle_pressed_in_last_msg = true;
	}
	else
	{
	  autopilot_toggle_pressed_in_last_msg = false;
	 }
	  
	 // disable autopliot if we have a non-zero input
	 if(!isApprox(twist.linear.x, 0.0, 1e-10) || !isApprox(twist.linear.y, 0.0, 1e-10) || !isApprox(twist.linear.z, 0.0, 1e-10) || !isApprox(twist.angular.z, 0.0, 1e-10))
	 {
	   autopilot_ = false;
	 }
//	 
	 if(old_autopilot_ != autopilot_)
	 {
	   ROS_INFO_COND(autopilot_, "controller enabled");
	   ROS_INFO_COND(!autopilot_, "controller disabled");
	   
	   std_msgs::Bool msg;
	   msg.data = autopilot_;
	   pub_enable_autopilot.publish(msg);
	 }
	 
	 // toggle only once!
	// if (!toggle_pressed_in_last_msg && emergency_toggle_pressed){
	//  ROS_INFO("Changing emergency status");
	//  pub_toggle_state.publish(std_msgs::Empty());
	// }
	// toggle_pressed_in_last_msg = emergency_toggle_pressed;


	// if (!cam_toggle_pressed_in_last_msg && cam_toggle_pressed){
	//  ROS_INFO("Changing Camera");
	//  if (!srv_cl_cam.call(srv_empty))  ROS_INFO("Failed to toggle Camera");
	// }
	// cam_toggle_pressed_in_last_msg = cam_toggle_pressed;



	}


	TeleopArDrone(){

	 twist.linear.x = twist.linear.y = twist.linear.z = 0;
	 twist.angular.x = twist.angular.y = twist.angular.z = 0;

	 is_flying = false;
	 got_first_joy_msg = false;

	 joy_sub = nh_.subscribe("/joy", 1,&TeleopArDrone::joyCb, this);
	 toggle_pressed_in_last_msg = cam_toggle_pressed_in_last_msg = false;

	 pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
	 pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
	 pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
	 pub_enable_autopilot = nh_.advertise<std_msgs::Bool>("/ardrone/enable_controller",1);
	 pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	// srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
	}


   template<typename T>
   bool isApprox(const T& v, const T& threshold, const T& tolerance)
   {
     return std::abs(v - threshold) < tolerance;
   }

	void send_cmd_vel() {
	  if(!autopilot_)
        pub_vel.publish(twist);
    }


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_teleop");

  ROS_INFO("Started ArDrone joystick-Teleop");
  ROS_INFO("Press and hold LB for takeoff");
  ROS_INFO("Press R1 to toggle emergency-state");
  ROS_INFO("Press 'select' to choose camera");
  ROS_INFO("Press 'start' to enable/disable the controller");

  TeleopArDrone teleop;
  ros::Rate pub_rate(PUBLISH_FREQ);

  while (teleop.nh_.ok())
  {
    ros::spinOnce();
    teleop.send_cmd_vel();
    pub_rate.sleep();
  }

  return 0;
}
