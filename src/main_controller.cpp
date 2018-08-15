#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "create_driver/create_driver.h"
#include <tf/transform_datatypes.h>
#include <sstream>
#include <boost/thread/thread.hpp>

int run = 1;
int left = 0;
int right = 0;
int wheelOK = 0;
float turn = 0.0;

ros::Subscriber sub;

void stopperCallback(const ca_msgs::Bumper& msg);

void endTimer(int *x, int value) {
  boost::this_thread::sleep_for(boost::chrono::seconds(1));
  *x = value;
  ros::NodeHandle n;
  sub = n.subscribe("bumper", 1000, stopperCallback);
  
}

void setTimer(int* x, int value) {
  sub.shutdown();
  int temp = *x;
  *x = value;
  boost::thread end (endTimer, x, temp);
}

void stopperCallback(const ca_msgs::Bumper& msg) {
  if(msg.is_left_pressed && msg.is_right_pressed) {
    ROS_INFO("Time to stop");
    run = 0;
  } else {
    if(msg.is_left_pressed) {
      setTimer(&left, 1);
    }
    else if(msg.is_right_pressed) {
      setTimer(&right, 1);
    }
    run = 1;
  }
  ros::NodeHandle n;
  return;
}

void wheeldropCallback(const std_msgs::Bool& msg) {
  if(msg.data) wheelOK = 0;
  else wheelOK = 1;
}

void lidarCallback(const std_msgs::Float32& msg) {
  turn = msg.data;
  ROS_INFO("%f",turn);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Subscriber wheelSub = n.subscribe("wheeldrop", 1000, wheeldropCallback);

  ros::Subscriber lidarSub = n.subscribe("lidar", 1000, lidarCallback);

  sub = n.subscribe("bumper", 1000, stopperCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  geometry_msgs::Twist msg;
  while (ros::ok())
  {
    if(run && wheelOK) {
      if(left) {
      ROS_INFO("left: %d", left);
        msg.linear.x = -0.3;
        msg.angular.z = -1.5;
      }
      else if(right) {
      ROS_INFO("right: %d", right);
        msg.linear.x = -0.3;
        msg.angular.z = 1.5;
      } else {
        msg.linear.x = 0.5;
        msg.angular.z = turn;
     }
    }
    else {
      msg.linear.x = 0.0;
      msg.angular.z = turn;
    }
    loop_rate.sleep();
    chatter_pub.publish(msg);
    ros::spinOnce();
  }
  return 0;
}
