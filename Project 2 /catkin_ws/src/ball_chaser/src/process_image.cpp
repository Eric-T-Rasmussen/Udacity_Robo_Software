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
  if (!client.call(srv)){
    ROS_ERROR("Failed to call service");
  }

}

//This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

  int white_pixel = 255;
  float left_range = 0;
  float forward_range = 0;
  float right_range = 0;

  left_range = img.step/3;
  forward_range = 2*img.step/3;
  right_range = img.step;

  int position;
  float angular, linear;
  position = 0;
  angular = 0.0;
  linear = 0.0;

  for (int i=0; i < img.height * img.step; i+= 3) {
    
    if ((img.data[i] == white_pixel) && (img.data[i+1] == white_pixel) && (img.data[i+2] == white_pixel)){
    
      position = i % img.step;
      //Drive commands after pixel scan

      //Left
      if (position  < left_range)
      {
        linear = 0.25;
        angular= 0.7;
        ROS_INFO_STREAM("going left");
        drive_robot(linear, angular);
        break;
      }

      //Forward
      else if (position > left_range && position <= forward_range)
      {
        linear = 0.5;
        angular = 0.0;
        ROS_INFO_STREAM("going forward");
        drive_robot(linear, angular);
        break;

      }

      //Right
      else if (position > forward_range)
      {
        linear = 0.25;
        angular = -0.7;
        ROS_INFO_STREAM("going right");
        drive_robot(linear, angular);
        break;
      }
    }
   
  
  }
  if (position==0){
    linear = 0.0;
    angular = 0.0;
    ROS_INFO_STREAM("No ball detected");
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