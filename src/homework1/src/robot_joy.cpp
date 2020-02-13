#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_,lineary_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_vrep;
  ros::Subscriber joy_sub_;

};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  lineary_(5),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_vrep = nh_.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 1); //vrep/twistCommand
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  twist.linear.y = l_scale_*joy->axes[lineary_]; // ajoute homework 4

  vel_pub_vrep.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robot");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
