#include "ros/ros.h"
#include "quadrotor_msgs/SO3Command.h"
#include "sensor_msgs/Joy.h"
#include "eigen3/Eigen/Geometry"
#include "nav_msgs/Odometry.h"
#include <so3_control/SO3Control.h>
#include <geometry_msgs/Point.h>

static SO3Control controller;

static double deadband = 0.1;  // deadband
static double scale = 2.0;

static bool positionControl_flag = false;
static bool takeoff_flag = false;
static bool land_flag = false;

Eigen::Vector3d des_pos, des_vel, des_acc, pos, vel;
geometry_msgs::Point target_point;
ros::Publisher target_pub;

void takeoff();
void land();


// odom callback 
void odomMsgCb(const nav_msgs::OdometryConstPtr &odom_msg)
{
  pos = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                        odom_msg->pose.pose.position.y,
 			odom_msg->pose.pose.position.z);

  vel = Eigen::Vector3d(odom_msg->twist.twist.linear.x, 
   			odom_msg->twist.twist.linear.y,
			odom_msg->twist.twist.linear.z);
  
  if (!positionControl_flag) 
  {
    des_pos = pos;
  }
  else
  { 
    if (takeoff_flag) 
    {
      des_pos = pos + Eigen::Vector3d(0,0,1);
      target_point.x = des_pos(0); 
      target_point.y = des_pos(1);
      target_point.z = des_pos(2);
      //target_pub.publish(target_point);
      ROS_INFO("Taking off!");
      takeoff_flag = false;
    }
    else if (land_flag) 
    {
      des_pos(2) = 0;
      target_point.x = des_pos(0); 
      target_point.y = des_pos(1);
      target_point.z = des_pos(2);
      //target_pub.publish(target_point);
      ROS_INFO("Landing!");
      land_flag = false;
    }
    if (fabs(des_pos(2) - pos(2))< 0.01)
    {
      positionControl_flag = false;
    }
  }
}

// joystick callback
void JoyMsgCb(const sensor_msgs::JoyConstPtr &joy_msg)
{

  if (joy_msg->buttons[2] == 1) takeoff();
  if (joy_msg->buttons[1] == 1) land();

  if (joy_msg->axes[5] < -0.1)
  {
    double v1, v2, v3, v4;
    v1 = joy_msg->axes[4];
    v2 = joy_msg->axes[3];
    v3 = joy_msg->axes[1];
    v4 = joy_msg->axes[0];

    // deadband check
    if (-deadband < v1 && deadband > v1) v1 = 0.0;
    if (-deadband < v2 && deadband > v2) v2 = 0.0;
    if (-deadband < v3 && deadband > v3) v3 = 0.0; 
    if (-deadband < v4 && deadband > v4) v4 = 0.0;

    // Command Velocity
    des_vel(0) = v1 * scale;
    des_vel(1) = v2 * scale;
    des_vel(2) = v3 * scale;
  }
  else 
  {
    des_vel = Eigen::Vector3d::Zero();
  }
}

void takeoff()
{
  positionControl_flag = true;
  takeoff_flag = true;
  ros::param::set("attack_node/NoiseFlag", true);
}

void land()
{
  positionControl_flag = true;
  land_flag = true;
  ros::param::set("attack_node/NoiseFlag", false);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "JoystickVelControlNode");
  ros::NodeHandle nh_;
  
  target_pub = nh_.advertise<geometry_msgs::Point>("target", 1);
  ros::Publisher  so3_command_pub = nh_.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &odomMsgCb);
  ros::Subscriber joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyMsgCb);

  // define control gains
  Eigen::Vector3d kx, kv;
  kx(0) = 3; kx(1) = 3; kx(2) = 6;
  kv(0) = 3; kv(1) = 3; kv(2) = 5;

  // SO3_Command Initialization
  quadrotor_msgs::SO3Command so3_command;
  so3_command.header.frame_id = "/quadrotor";  
  so3_command.kR[0] = 0.5;
  so3_command.kR[1] = 0.5;
  so3_command.kR[2] = 0.5;
  so3_command.kOm[0] = 0.1;
  so3_command.kOm[1] = 0.1;
  so3_command.kOm[2] = 0.1;
/*
  so3_command.angular.x = 0;
  so3_command.angular.y = 0;
  so3_command.angular.z = 0;
  so3_command.aux.current_yaw = 0;
  so3_command.aux.kf_correction = 0;
  so3_command.aux.angle_corrections[0] = 0;
  so3_command.aux.angle_corrections[1] = 0;
  so3_command.aux.enable_motors = true;
  so3_command.aux.use_external_yaw = true;
*/
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    loop_rate.sleep();
    // calculate control 
    controller.setPosition(pos);
    controller.setVelocity(vel);
    controller.calculateControl(des_pos, des_vel, des_acc, 0, 0, kx, kv); 
    Eigen::Vector3d force = controller.getComputedForce();
    Eigen::Quaterniond orientation = controller.getComputedOrientation();

    // Header time
    so3_command.header.stamp = ros::Time::now();
    so3_command.force.x = force(0);
    so3_command.force.y = force(1);
    so3_command.force.z = force(2);
    so3_command.orientation.x = orientation.x();
    so3_command.orientation.y = orientation.y();
    so3_command.orientation.z = orientation.z();
    so3_command.orientation.w = orientation.w(); 
  
    // publish
    so3_command_pub.publish(so3_command);
    target_pub.publish(target_point);
    ros::spinOnce();
  }
  return 0;
}


