#include "ros/ros.h"
#include "quadrotor_msgs/LinearCommand.h"
#include "sensor_msgs/Joy.h"
#include "eigen3/Eigen/Geometry"
#include "nav_msgs/Odometry.h"
#include "Linear_Control.h"

static LinearControl controller;

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
  Eigen::Vector3d omega = Eigen::Vector3d(odom_msg->twist.twist.angular.x,
					  odom_msg->twist.twist.angular.y,
					  odom_msg->twist.twist.angular.z);
  Eigen::Quaterniond q = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z);
  // convert quaternion to rotation matrix
  Eigen::Matrix3d R = q.toRotationMatrix();
  // get roll, pitch, yaw ----- Euler Z-X-Y
  Eigen::Vector3d euler = Eigen::Vector3d(asin(R(2,1)),
                                          atan2(-R(2,0), R(2,2)),
                                          atan2(-R(0,1), R(1,1)));
  // set current state
  //controller.setPosition(pos);
  //controller.setVelocity(vel);
  //controller.setAngleVel(omega);
  //controller.setEuler(euler);
  
  if (!positionControl_flag) des_pos = pos;
  else
  { 
    if (takeoff_flag) 
    {
      des_pos = pos + Eigen::Vector3d(0,0,1);
      target_point.x = des_pos(0); 
      target_point.y = des_pos(1);
      target_point.z = des_pos(2);
      //target_pub.publish(target_point);
      ROS_INFO("Takeoff!");
      takeoff_flag = false;
    }
    else if (land_flag) 
    {
      des_pos(2) = 0;
      target_point.x = des_pos(0); 
      target_point.y = des_pos(1);
      target_point.z = des_pos(2);
      //target_pub.publish(target_point);
      ROS_INFO("Land");
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
  ros::param::set("attack_node/NoiseFlag", true);
  takeoff_flag = true;
}

void land()
{
  positionControl_flag = true;
  ros::param::set("attack_node/NoiseFlag", false);
  land_flag = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "JoystickLinearVelControlNode");
  ros::NodeHandle nh_;
  
  target_pub = nh_.advertise<geometry_msgs::Point>("target", 1);
  ros::Publisher  linear_command_pub = nh_.advertise<quadrotor_msgs::LinearCommand>("linear_cmd", 10);
  ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &odomMsgCb);
  ros::Subscriber joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyMsgCb);

  // define position control gains
  Eigen::Vector3d kx, kv;
  kx(0) = 5;   kx(1) = 5;   kx(2) = 10;
  kv(0) = 2.5; kv(1) = 2.5; kv(2) = 8;

  // Linear_Command Initialization
  quadrotor_msgs::LinearCommand linear_command;
  linear_command.header.frame_id = "/quadrotor"; 
  // define roll, pitch, yaw control gains
  /*linear_command.kR[0] = 0.2665;  
  linear_command.kR[1] = 0.212;    
  linear_command.kR[2] = 1.1037;
  linear_command.kOm[0] = 0.071;  
  linear_command.kOm[1] = 0.0711;  
  linear_command.kOm[2] = 0.2453;
*/
  linear_command.kR[0] = 1.2665;  
  linear_command.kR[1] = 0.7412;    
  linear_command.kR[2] = 1.1037;
  linear_command.kOm[0] = 0.2111;  
  linear_command.kOm[1] = 0.2111;  
  linear_command.kOm[2] = 0.2453; 

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    loop_rate.sleep();
    // calculate control 
    controller.setPosition(pos);
    controller.setVelocity(vel);
    controller.calculateControl(des_pos, des_vel, des_acc, 0, 0, kx, kv, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    Eigen::Vector3d force = controller.getComputedForce();
    Eigen::Quaterniond orientation = controller.getComputedOrientation();

    // Header time
    linear_command.header.stamp = ros::Time::now();
    linear_command.force.x = force(0);
    linear_command.force.y = force(1);
    linear_command.force.z = force(2);
    linear_command.orientation.x = orientation.x();
    linear_command.orientation.y = orientation.y();
    linear_command.orientation.z = orientation.z();
    linear_command.orientation.w = orientation.w(); 
  
    // publish
    linear_command_pub.publish(linear_command);
    target_pub.publish(target_point);
    ros::spinOnce();
  }
  return 0;
}


