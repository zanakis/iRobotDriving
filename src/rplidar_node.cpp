#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

ros::Publisher pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int count = scan->scan_time / scan->time_increment;
  std_msgs::Float32 msg;
  msg.data = 0.0;
  for(int i = 0; i < count; i++) {
    if(RAD2DEG(scan->angle_min + scan->angle_increment * i) > 120) {
      if(scan->ranges[i]<0.5) {
        ROS_INFO ("right %f", RAD2DEG(scan->angle_min + scan->angle_increment * i));

        msg.data = 1.50;
      }
    }  if(RAD2DEG(scan->angle_min + scan->angle_increment * i) < -120) {
      if(scan->ranges[i]<0.5) {
        ROS_INFO ("left %f", RAD2DEG(scan->angle_min + scan->angle_increment * i));

        msg.data = -1.50;
      }
    }
  }
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rplidar_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

  pub = n.advertise<std_msgs::Float32>("lidar", 1000);

  ros::spin();

  return 0;
}
