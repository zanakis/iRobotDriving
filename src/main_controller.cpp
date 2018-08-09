#include "ros/ros.h"
#include "std_msgs/String.h"
#include "create_driver/create_driver.h"
#include <tf/transform_datatypes.h>
#include <sstream>

int run = 1;

void stopperCallback(const ca_msgs::Bumper& msg) {
  if(msg.is_left_pressed && msg.is_right_pressed) {
    ROS_INFO("Time to stop");
    run = 0;
  } else run = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);


  ros::Subscriber sub = n.subscribe("bumper", 1000, stopperCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  geometry_msgs::Twist msg;
  while (ros::ok())
  {
    if(run) {
      msg.linear.x = 0.5;
      msg.angular.z = 0.0;
    }
    else {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }
    loop_rate.sleep();
    chatter_pub.publish(msg);
    ros::spinOnce();
  }

  return 0;
}