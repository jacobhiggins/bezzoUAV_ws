#include "ros/ros.h"
#include "quadrotor_msgs/LinearCommand.h"
#include "sensor_msgs/Joy.h"
#include "eigen3/Eigen/Geometry"

static double scale_f = 15.0;  // thrust scale
static double scale_rpy = 0.5; // rpy scale
static double deadband = 0.1;  // deadband
static bool alt_flag = false;
static double mass_ = 1.282;
static double g_ = 9.81;
static double thrust, roll, pitch, yaw;

void JoyMsgCb(const sensor_msgs::JoyConstPtr &joy_msg)
{
  double v0, v1, v3, v4;
  v3   = -joy_msg->axes[3];
  v4  = joy_msg->axes[4];
  v1 = joy_msg->axes[1];
  v0    = joy_msg->axes[0];

  // deadband check
  if (v1 < deadband) v1 = 0.0;
  if (-deadband < v3 && deadband > v3)   v3 = 0.0;
  if (-deadband < v4 && deadband > v4)   v4 = 0.0; 
  if (-deadband < v0 && deadband > v0)   v0 = 0.0;
  
  // scale joystick input
  roll = v3 * scale_rpy;
  pitch = v4 * scale_rpy;
  //yaw = v0 * scale_rpy;
  yaw = 0;
  thrust = v1 * scale_f;

  // Altitude hold flag check
  //if (joy_msg->buttons[0] == 1 && thrust > deadband) alt_flag = true;
  //else if (joy_msg->buttons[0] == 1 && thrust < deadband) alt_flag = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "JoystickTRPYControlNode");
  ros::NodeHandle nh_;
  
  ros::Subscriber joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyMsgCb);
  ros::Publisher linear_command_pub = nh_.advertise<quadrotor_msgs::LinearCommand>("linear_cmd", 10);

  // Linear_Command Initialization
  quadrotor_msgs::LinearCommand linear_command;
  linear_command.header.frame_id = "/quadrotor";  
  // define roll, pitch, yaw control gains
  linear_command.kR[0] = 0.2665;  
  linear_command.kR[1] = 0.212;    
  linear_command.kR[2] = 1.1037;
  linear_command.kOm[0] = 0.071;  
  linear_command.kOm[1] = 0.0711;  
  linear_command.kOm[2] = 0.2453; 

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();

    // Header time
    linear_command.header.stamp = ros::Time::now();
    linear_command.force.x = 0;
    linear_command.force.y = 0;
    linear_command.force.z = mass_*g_;
    // roll pitch yaw
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond orientation = yawAngle * rollAngle * pitchAngle;
    linear_command.orientation.x = orientation.x();
    linear_command.orientation.y = orientation.y();
    linear_command.orientation.z = orientation.z();
    linear_command.orientation.w = orientation.w(); 
  
    // publish
    linear_command_pub.publish(linear_command);

    ros::spinOnce();
  }
  return 0;
}


