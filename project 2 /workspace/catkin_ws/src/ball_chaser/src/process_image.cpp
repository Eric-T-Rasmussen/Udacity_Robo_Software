#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

//Define a global client that can request services
ros::ServiceClient client;


//This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // request a service
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

}

//This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

  int white_pixel = 255;
  int left_range = 0;
  int forward_range = 0;
  int right_range = 0;

  left_range = img.width/3;
  forward_range = 2*img.width/3;
  right_range = img.width;

  int position, angular, linear;
  position = 0;
  angular = 0;
  linear = 0;

  for (int i=0; i < img.height * img.step; i+= 3) {
    
    if ((img.data[i] == white_pixel) && (img.data[i+1] == white_pixel) && (img.data[i+2] == white_pixel)){
    
      position = i % img.width;
      //Drive commands after pixel scan

      //Left
      if (position  < left_range)
      {
        linear = 0.0;
        angular= 0.5;
        ROS_INFO_STREAM("going left");
      }

      //Forward
      else if (position > left_range && position <= forward_range)
      {
        linear = 0.5;
        angular = 0.0;
        ROS_INFO_STREAM("going forward");

      }

      //Right
      else if (position > forward_range)
      {
        linear = 0.0;
        angular = -0.5;
        ROS_INFO_STREAM("going right");
      }
  
      else
      {
        linear = 0.0;
        angular = 0.0;
        ROS_INFO_STREAM("No ball detected");
      }
    }

    drive_robot(linear, angular);
  }

  
  
}

int main(int argc, char** argv)
{
  //Intialize
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  //Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  //subscribe to /camera/rgb/image_raw topic to read the image data iside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  //Handle ROS communication events
  ros::spin();

  return 0;
}