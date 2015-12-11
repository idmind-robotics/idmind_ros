#ifndef IDMIND_BEHAVIOURS_H
#define IDMIND_BEHAVIOURS_H

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

struct Colour
{
  int red;
  int green;
  int blue;
};

struct Behaviour
{
  int head;
  int left_arm;
  int right_arm;
  int mouth;
  std::vector<int> leds;
};

class IdmindBehaviours
{
public:
  IdmindBehaviours();
//  ~IdmindBehaviours();

  void example();

private:
  void sendBehaviour(int i);

  void setHead(int position, int velocity);
  void setMouth(int mouth);
  void setLight(int light, int red, int green, int blue, int time);
  void setArms(int pos_left, int time_left, int pos_right, int time_right);

  bool readFiles();

  template<typename T>
  bool readNumbersFile(std::string file, std::string folder, std::vector<std::vector<T> >* vector);

  ros::NodeHandle n_;
  ros::Publisher head_pub_, mouth_pub_, lights_pub_, arms_pub_;

  std::vector<Colour> colours_;
  std::vector<Behaviour> behaviours_;

  std::string behaviours_folder_;
};

#endif
