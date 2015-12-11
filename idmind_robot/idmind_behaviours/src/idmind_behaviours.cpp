#include "idmind_behaviours/idmind_behaviours.h"

IdmindBehaviours::IdmindBehaviours()
    : n_("~"), behaviours_folder_("/catkin_ws/src/no_build/config/behaviours/")
{
  head_pub_ = n_.advertise<std_msgs::UInt8MultiArray>("/idmind_interaction/head", 100);
  mouth_pub_ = n_.advertise<std_msgs::UInt8>("/mouth_encoder/mouth_shape", 100);
  lights_pub_ = n_.advertise<std_msgs::UInt8MultiArray>("/idmind_interaction/lights", 100);
  arms_pub_ = n_.advertise<std_msgs::UInt16MultiArray>("/idmind_arms/position", 100);

  ros::Time now = ros::Time::now();
  while (true)
  {
    if (head_pub_.getNumSubscribers() != 0 &&
        mouth_pub_.getNumSubscribers() != 0 &&
        lights_pub_.getNumSubscribers() != 0 &&
        arms_pub_.getNumSubscribers() != 0)
      break;

    if (ros::Time::now() > now + ros::Duration(1.0))
    {
      ROS_ERROR("%s --> One or more topics were not subscribed to.", ros::this_node::getName().c_str());
      break;
    }

    ros::Duration(0.010).sleep();
  }
}

void IdmindBehaviours::example()
{
  readFiles();

  sendBehaviour(1);
  ros::Duration(3.0).sleep();

  sendBehaviour(2);
  ros::Duration(3.0).sleep();

  sendBehaviour(3);
  ros::Duration(3.0).sleep();

  sendBehaviour(0);
}

void IdmindBehaviours::sendBehaviour(int i)
{
  setHead(behaviours_[i].head, 40);

  setMouth(behaviours_[i].mouth);

  for (int j=0; j<6; ++j)
  {
    if (j != 2)
      setLight(j, colours_[behaviours_[i].leds[j]].red,
                  colours_[behaviours_[i].leds[j]].green,
                  colours_[behaviours_[i].leds[j]].blue, 10);
  }

  setArms(behaviours_[i].left_arm, 1500, behaviours_[i].right_arm, 1500);
}

void IdmindBehaviours::setHead(int position, int velocity)
{
  std_msgs::UInt8MultiArray msg;
  msg.data.resize(2);
  msg.data[0] = position;
  msg.data[1] = velocity;
  head_pub_.publish(msg);
}
void IdmindBehaviours::setMouth(int mouth)
{
  std_msgs::UInt8 msg;
  msg.data = mouth;
  mouth_pub_.publish(msg);
}
void IdmindBehaviours::setLight(int light, int red, int green, int blue, int time)
{
  std_msgs::UInt8MultiArray msg;
  msg.data.resize(5);
  msg.data[0] = light;
  msg.data[1] = red;
  msg.data[2] = green;
  msg.data[3] = blue;
  msg.data[4] = time;
  lights_pub_.publish(msg);
}
void IdmindBehaviours::setArms(int pos_left, int time_left, int pos_right, int time_right)
{
  std_msgs::UInt16MultiArray msg;
  msg.data.resize(4);
  msg.data[0] = pos_left;
  msg.data[1] = time_left;
  msg.data[2] = pos_right;
  msg.data[3] = time_right;
  arms_pub_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "idmind_behaviours");

  IdmindBehaviours id_be;
  id_be.example();

  return 0;
}
