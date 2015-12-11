#include "idmind_sensors/idmind_sensors.h"

IdmindSensors::IdmindSensors()
    : n_("~"), serial_("/dev/idmind-sensorsboard", 115200, 5), ground_min_(4, 0),
      frequency_(0.0), green_light_(false), reconnecting_(false)
{
  ground_sensors_pub_ = n_.advertise<std_msgs::Int32MultiArray>("ground_sensors", 100);
  twist_pub_ = n_.advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);
  power_button_pub_ = n_.advertise<std_msgs::UInt8>("power_button", 100);

  batteries_serv_ = n_.advertiseService("batteries", &IdmindSensors::batteriesService, this);
  dock_undock_serv_ = n_.advertiseService("dock_undock", &IdmindSensors::dockUndockService, this);

  hardstop_client_ = n_.serviceClient<idmind_motors::Hardstop>("/idmind_motors/hardstop", true);
//  hardstop_client_.waitForExistence();

  power_button_check_ = n_.createTimer(ros::Duration(1.0), &IdmindSensors::powerButtonCheck, this);
  service_check_ = n_.createTimer(ros::Duration(0.1), &IdmindSensors::serviceCheck, this);

  bool params(true);

  ros::param::param("~fall_hardstop", fall_hardstop_, false);

  params &= ros::param::get("~ground_min_left", ground_min_[0]);
  params &= ros::param::get("~ground_min_front_left", ground_min_[1]);
  params &= ros::param::get("~ground_min_front_right", ground_min_[2]);
  params &= ros::param::get("~ground_min_right", ground_min_[3]);

  last_hardstop_ = ros::Time::now();

  batteries_command_ = 0x51;
  aux_batteries_command_ = 0x52;
  charger_status_command_ = 0x58;
  ground_sensors_command_ = 0x59;

  if (serial_.ready_ && ((fall_hardstop_ && params) || !fall_hardstop_))
  {
    ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
    green_light_ = true;
  }
  else
    ROS_ERROR("%s ---> FAILED <---", ros::this_node::getName().c_str());
}

void IdmindSensors::runPeriodically()
{
  frequency_ = 20.0;

  ros::Rate r(frequency_);
  while(n_.ok())
  {
    ros::spinOnce();

    if (reconnecting_ && hardstop_client_.exists())
    {
      hardstop_client_.shutdown();
      hardstop_client_ = n_.serviceClient<idmind_motors::Hardstop>("/idmind_motors/hardstop", true);
      ROS_INFO("%s --> Reconnected to hardstop service server.", ros::this_node::getName().c_str());
      reconnecting_ = false;
    }

    readPublishGroundSensors();

    r.sleep();
  }
}

bool IdmindSensors::batteriesService(idmind_sensors::Batteries::Request& req, idmind_sensors::Batteries::Response& res)
{
  std::vector<double> batteries(5, 0.0);
  res.success = readBatteries(&batteries);

  res.motors_battery = batteries[0];
  res.electronics_battery = batteries[1];
  res.charger_voltage = batteries[2];
  res.aux_1_voltage = batteries[3];
  res.aux_2_voltage = batteries[4];

  int byte = readChargerStatus();

  if (byte >= 0)
  {
    res.electronics_charging = (byte & 0x01) != 0 ? 1 : 0;
    res.motors_charging = (byte & 0x02) != 0 ? 1 : 0;
    res.aux_1_charging = (byte & 0x04) != 0 ? 1 : 0;
    res.aux_2_charging = (byte & 0x08) != 0 ? 1 : 0;
    res.power_button = (byte & 0x80) != 0 ? 1 : 0;

//    ROS_INFO("%d | %d 0 0 0 %d %d %d %d", byte, res.power_button, res.aux_2_charging, res.aux_1_charging,
//        res.motors_charging, res.electronics_charging);
  }
  else
    res.success = false;

  return res.success;
}

bool IdmindSensors::readBatteries(std::vector<double>* batteries)
{
  if (serial_.write((uint8_t)batteries_command_))
  {
    uint8_t buffer[7];
    if (serial_.read(buffer, 7, true) && buffer[0] == batteries_command_)
      for (int i = 0; i < 3; i++)
        (*batteries)[i] = static_cast<double>(buffer[i+1]) / 10.0;
    else
    {
      ROS_ERROR("%s --> Failed to read main batteries.", ros::this_node::getName().c_str());
      return false;
    }
  }

  if (serial_.write((uint8_t)aux_batteries_command_))
  {
    uint8_t buffer[6];
    if (serial_.read(buffer, 6, true) && buffer[0] == aux_batteries_command_)
      for (int i = 3; i < 5; i++)
        (*batteries)[i] = static_cast<double>(buffer[i-2]) / 10.0;
    else
    {
      ROS_ERROR("%s --> Failed to read auxiliary batteries.", ros::this_node::getName().c_str());
      return false;
    }
  }

  return true;
}

void IdmindSensors::powerButtonCheck(const ros::TimerEvent&)
{
  int byte = readChargerStatus();

  if (byte >= 0)
  {
    std_msgs::UInt8 power_button;
    power_button.data = (byte & 0x80) != 0 ? 1 : 0;

    power_button_pub_.publish(power_button);
  }
}

int IdmindSensors::readChargerStatus()
{
  if (serial_.write((uint8_t)charger_status_command_))
  {
    uint8_t buffer[5];
    if (serial_.read(buffer, 5, true) && buffer[0] == charger_status_command_)
      return static_cast<int>(buffer[1]);
  }

  ROS_ERROR("%s --> Failed to read charger status.", ros::this_node::getName().c_str());
  return -1;
}

bool IdmindSensors::dockUndockService(idmind_sensors::DockUndock::Request& req, idmind_sensors::DockUndock::Response& res)
{
  // 1 - dock / 2 - undock & exit / 3 - enter & dock
  if (req.control < 1 || req.control > 3)
    return false;

  int sum = 0;

  if (req.control == 1)
  {
    sum += sendCommandControl(0x45, 1);
    sum += sendCommandControl(0x40, 1);
    sum += sendCommandControl(0x41, 1);
    sum += sendCommandControl(0x46, 1);
  }
  else if (req.control == 2)
  {
    sum += sendCommandControl(0x45, 2);
    sum += sendCommandControl(0x46, 2);
    sum += sendCommandControl(0x40, 2);
    sum += sendCommandControl(0x41, 2);
  }

  if (req.control != 3 && sum != 4)
    return false;

  if (req.control == 1)
    return true;

  geometry_msgs::Twist twist;
  twist.linear.x = req.control == 2 ? 0.4 : -0.08;

  double wait_time = req.control == 2 ? 1.5 : 8.0;
  ros::Time now = ros::Time::now();

  while (ros::Time::now() < now + ros::Duration(wait_time))
  {
    twist_pub_.publish(twist);
    ros::Duration(1/frequency_).sleep();
  }

  twist.linear.x = 0.0;
  twist_pub_.publish(twist);
  ros::Duration(1.0).sleep();

  std::vector<double> batteries(5, 0.0);
  if (readBatteries(&batteries))
    if ((req.control == 2 && batteries[2] == 0) ||
        (req.control == 3 && batteries[2] > 17.0))
    {
      ROS_INFO("%s --> %s successfully.", ros::this_node::getName().c_str(), req.control == 2 ? "Undocked" : "Docked");
      res.success = true;
    }
    else
    {
      ROS_INFO("%s --> Could not %s.", ros::this_node::getName().c_str(), req.control == 2 ? "undock" : "dock");
      res.success = false;
    }

  return true;
}

int IdmindSensors::sendCommandControl(int command, int control)
{
  uint8_t msg[2];
  msg[0] = (uint8_t)command;
  msg[1] = (uint8_t)control;

  if (serial_.write(msg, 2))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || !(buffer[0] == command))
    {
      ROS_ERROR("%s --> Failed to send dock/undock 0x%X %d.", ros::this_node::getName().c_str(), command, control);
      return 0;
    }
  }

  ros::Duration(control == 2 ? 0.01 : 0.5).sleep();

  return 1;
}

void IdmindSensors::serviceCheck(const ros::TimerEvent&)
{
  if ((!hardstop_client_.isValid() || !hardstop_client_.exists()) && !reconnecting_)
  {
    ROS_INFO("%s --> Lost connection to hardstop service server. Reconnecting...", ros::this_node::getName().c_str());
    reconnecting_ = true;
  }
}

void IdmindSensors::sendHardstop()
{
  if (ros::Time::now() > last_hardstop_ + ros::Duration(2.0))
  {
    idmind_motors::Hardstop msg;
    msg.request.activate = 1;
    msg.request.hardstop_time = 2;
    msg.request.back_up = 0;
    msg.request.stay_blocked = 0;
    hardstop_client_.call(msg);
    last_hardstop_ = ros::Time::now();
  }
}

void IdmindSensors::readPublishGroundSensors()
{
  if (serial_.write((uint8_t)ground_sensors_command_))
  {
    uint8_t buffer[8];
    if (serial_.read(buffer, 8, true) && buffer[0] == ground_sensors_command_)
    {
      std_msgs::Int32MultiArray ground_sensors;
      ground_sensors.data.resize(4, 0);

      for (int i=0; i<4; i++)
      {
        ground_sensors.data[3-i] = static_cast<int>(buffer[i+1]); //Left to right.

        if (ground_sensors.data[3-i] < ground_min_[3-i])
        {
//          ground_min_[3-i] = ground_sensors.data[3-i];

          if (fall_hardstop_)
            sendHardstop();
        }
      }

//      ROS_INFO("%d | %d | %d | %d", ground_min_[0], ground_min_[1],
//          ground_min_[2], ground_min_[3]);

      ground_sensors_pub_.publish(ground_sensors);
    }
    else
      ROS_ERROR("%s --> Failed to read ground sensors.", ros::this_node::getName().c_str());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "idmind_sensors");

  IdmindSensors sensors;

  if (sensors.green_light_)
    sensors.runPeriodically();

  return 0;
}
