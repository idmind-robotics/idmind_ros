#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>

namespace
{
sensor_msgs::LaserScan scan;
std_msgs::UInt8 status;

std::string scan_topic, status_topic;

double arc_angle, distance_threshold;
double PI(3.1415926536);

bool params(true), new_scan(false);

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  scan = *msg;
  new_scan = true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_obstacle_detector");

  ros::NodeHandle n("~");

  params &= ros::param::get("~scan_topic", scan_topic);
  params &= ros::param::get("~status_topic", status_topic);

  if (!params)
  {
    ROS_ERROR("%s --> Failed to get essential parameters.", ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>((std::string("/") + scan_topic).c_str(), 100, &scanCallback);
  ros::Publisher status_pub = n.advertise<std_msgs::UInt8>(status_topic.c_str(), 100);

  ros::param::param("~arc_angle", arc_angle, 80.0);
  ros::param::param("~distance_threshold", distance_threshold, 0.4);

  ros::Rate r(20);
  while (ros::ok())
  {
    ros::spinOnce();

    if (new_scan)
    {
      status.data = 0;

      double arc_angle_rad = arc_angle * PI / 180;
      double arc_start = -arc_angle_rad/2 > scan.angle_min ? -arc_angle_rad/2 : scan.angle_min;
      double arc_end = arc_angle_rad/2 < scan.angle_max ? arc_angle_rad/2 : scan.angle_max;

      int first_scan = (arc_start - scan.angle_min) / scan.angle_increment;
      int last_scan = (arc_end - scan.angle_min) / scan.angle_increment;

      for (int i=first_scan; i<=last_scan; ++i)
      {
        if (scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
          continue;

        if (scan.ranges[i] < distance_threshold)
        {
//          ROS_WARN("Distance: %.2f | Angle: %.2f | Scan: %d", scan.ranges[i], scan.angle_min + scan.angle_increment*i, i);
          status.data = 1;
        }
      }

      status_pub.publish(status);

      new_scan = false;
    }

    r.sleep();
  }

  return 0;
}
