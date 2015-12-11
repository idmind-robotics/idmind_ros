#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "idmind_utilities/PoseInit.h"

namespace
{
geometry_msgs::PoseWithCovarianceStamped initial_pose;

bool publish = false;

bool poseInitService(idmind_utilities::PoseInit::Request& req, idmind_utilities::PoseInit::Response& res)
{
  initial_pose.pose.pose.position.x = req.x;
  initial_pose.pose.pose.position.y = req.y;
  initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(req.th);

  publish = true;
  return true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_initializer");

  ros::NodeHandle n("~");
  ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  ros::ServiceServer pose_init_serv = n.advertiseService("pose_init", &poseInitService);

  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp =  ros::Time::now();
  initial_pose.pose.covariance.elems[0] = 1;
  initial_pose.pose.covariance.elems[7] = 1;
  initial_pose.pose.covariance.elems[35] = 1;

  ros::Rate r(30);
  while (ros::ok())
  {
    ros::spinOnce();

    if (publish)
    {
      initial_pose_pub.publish(initial_pose);
      publish = false;
    }

    r.sleep();
  }

  return 0;
}
