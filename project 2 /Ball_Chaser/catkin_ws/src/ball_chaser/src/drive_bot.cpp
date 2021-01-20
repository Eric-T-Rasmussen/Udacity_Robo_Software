#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

//Ros::Publisher motor commands;
ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{

  ROS_INFO("DriveToTarget Request received -- linear velocity:%1.2f, angular velocity:1.2f", (float)req.linear_x, (float)req.angular_z);

  
  std_msgs::Float64 lin_x, ang_z;
  lin_x.data = req.linear_x;
  ang_z.data = req.angular_z;
  geometry_msgs::Twist motor_command;
  motor_command.linear.x= (double)lin_x.data;
  motor_command.angular.z = (double)ang_z.data;

  //motor command taking both linear_x and angular_z velocities
  motor_command_publisher.publish(motor_command);

	res.msg_feedback = "Velocities set - linear velocity: " + std::to_string(lin_x.data) + "angular velocity: " + std::to_string(ang_z.data);

	ROS_INFO_STREAM(res.msg_feedback);

	return true;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_bot");

	//ROS nodehandle object
	ros::NodeHandle n;

	//Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation 
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	ros::ServiceServer service=n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

	ROS_INFO("Ready to send joint commands");

	ros::spin();
	return 0;

}