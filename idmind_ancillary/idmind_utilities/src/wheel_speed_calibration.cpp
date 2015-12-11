#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

namespace
{
std_msgs::Int32MultiArray encoders;
geometry_msgs::Twist twist;
ros::Time last_time;

std::string idmind_ns("idmind_robot/"), kinematics;

const double PI = 3.1415926536;

double dt, d, w, w_avg;
double increase = 1.0, factor = 0.0, w_target = 1;

int ticks;
int key = 0;

bool new_encoder = false;

void encodersCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  ros::Time current_time = ros::Time::now();
  dt = (current_time - last_time).toSec();
  last_time = current_time;
  encoders = *msg;
  new_encoder = true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_speed_calibration");

  ros::NodeHandle n("~");
  ros::Subscriber encoders_sub = n.subscribe<std_msgs::Int32MultiArray>("/idmind_motors/encoders", 100, &encodersCallback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);

  ros::param::get(idmind_ns+"kinematics", kinematics);
  ros::param::get(idmind_ns+"ticks", ticks);

  int wheel = atoi(kinematics.substr(18).c_str()) - 1;

  ros::Rate r(40);

  last_time = ros::Time::now();

  int samples = 20;
  double vels[samples];

  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);
  tcgetattr( STDIN_FILENO, &newt);
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
//  newt.c_cc[VTIME] = 0;

  while (ros::ok())
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    key = getchar();

    ros::spinOnce();

    if (new_encoder)
    {
      d = encoders.data[wheel] * 2 * PI / ticks;
      w = d/dt;

      w_avg = 0.0;
      for (int j=samples-1; j>0; j--)
      {
        w_avg += vels[j-1];
        vels[j] = vels[j-1];
      }
      vels[0] = w;
      w_avg += w;
      w_avg /= samples;

      new_encoder = false;
    }

    if (key == 119) //w
      factor += increase;
    if (key == 115) //s
      factor -= increase;
    if (key == 97) //a
      increase /= 10;
    if (key == 100) //d
      increase *= 10;

    if (key == 101) //e
      w_target += 0.1;
    if (key == 113) //q
      w_target -= 0.1;

    twist.linear.x = factor * w_target;

    ROS_INFO("dt= %.3f | w= %.3f | w_avg= %.3f | w_target= %.3f | factor= %.3f | increase= %.3f", dt, w, w_avg, w_target, factor, increase);

    twist_pub.publish(twist);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    r.sleep();
  }

  return 0;
}

