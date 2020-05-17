#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/LinearCommand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Geometry>
#include "Linear_Control.h"
#include <tf/transform_datatypes.h>

static LinearControl controller;
static ros::Publisher linear_command_pub;
static quadrotor_msgs::LinearCommand linear_command;
static bool position_cmd_updated = false, position_cmd_init = false;
static Eigen::Vector3d des_pos, des_vel, des_acc, kx, kv;
static double des_yaw = 0, des_yaw_dot = 0;
static double current_yaw = 0;
static bool enable_motors = false;
static bool use_external_yaw = true;
static bool use_angle_corrections = false;
static double kf_correction, angle_corrections[2];

static void publishLinearCommand(void)
{
  controller.calculateControl(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot,
                              kx, kv, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const Eigen::Vector3d force = controller.getComputedForce();
  const Eigen::Quaterniond &orientation = controller.getComputedOrientation();
  const Eigen::Vector3d &moment = controller.getComputedMoment();

  linear_command.header.stamp = ros::Time::now();
  linear_command.force.x = force(0);
  linear_command.force.y = force(1);
  linear_command.force.z = force(2);
  //linear_command.moment.x = moment(0);
  //linear_command.moment.y = moment(1);
  //linear_command.moment.z = moment(2);
  linear_command.orientation.x = orientation.x();
  linear_command.orientation.y = orientation.y();
  linear_command.orientation.z = orientation.z();
  linear_command.orientation.w = orientation.w();
  linear_command_pub.publish(linear_command);
}

static void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                            cmd->acceleration.z);
 // kx = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  //kv = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw = cmd->yaw;
  des_yaw_dot = cmd->yaw_dot;
  position_cmd_updated = true;
  position_cmd_init = true;

  //publishLinearCommand();
}

static void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);
  const Eigen::Vector3d omega(odom->twist.twist.angular.x,
                              odom->twist.twist.angular.y,
                              odom->twist.twist.angular.z);

  Eigen::Quaterniond q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                            odom->pose.pose.orientation.x,
                                            odom->pose.pose.orientation.y,
                                            odom->pose.pose.orientation.z);
  // convert quaternion to rotation matrix
  Eigen::Matrix3d R = q.toRotationMatrix();
  // get roll, pitch, yaw ----- Euler Z-X-Y
  Eigen::Vector3d euler = Eigen::Vector3d(asin(R(2,1)),
                                          atan2(-R(2,0), R(2,2)),
                                          atan2(-R(0,1), R(1,1)));

  current_yaw = tf::getYaw(odom->pose.pose.orientation);

  controller.setPosition(position);
  controller.setVelocity(velocity);
  controller.setAngleVel(omega);
  controller.setEuler(euler);

  if(position_cmd_init)
  {
    // We set position_cmd_updated = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    if(!position_cmd_updated)
      publishLinearCommand();
    position_cmd_updated = false;
  }

}

static void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors = msg->data;
}

static void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg)
{
  kf_correction = msg->kf_correction;
  angle_corrections[0] = msg->angle_corrections[0];
  angle_corrections[1] = msg->angle_corrections[1];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_control");

  ros::NodeHandle n("~");

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  linear_command.header.frame_id = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 1.282);
  controller.setMass(mass);

  n.param("use_external_yaw", use_external_yaw, true);
  n.param("use_angle_corrections", use_angle_corrections, false);
 /*      kp_roll: 1.2665
       kd_roll: 0.2111
      kp_pitch: 0.7412
      kd_pitch: 0.2111
        kp_yaw: 1.1037
        kd_yaw: 0.2453
*/  
  // define roll, pitch, yaw control gains
  /*linear_command.kR[0] = 0.2665;  
  linear_command.kR[1] = 0.212;    
  linear_command.kR[2] = 1.1037;
  linear_command.kOm[0] = 0.071;  
  linear_command.kOm[1] = 0.0711;  
  linear_command.kOm[2] = 0.2453;
*/

  // low-level controller gains
  linear_command.kR[0] = 1.2665;  
  linear_command.kR[1] = 0.7412;    
  linear_command.kR[2] = 1.1037;
  linear_command.kOm[0] = 0.2111;  
  linear_command.kOm[1] = 0.2111;  
  linear_command.kOm[2] = 0.2453;

  // high-level controller gains
  kx(0) = 5;
  kx(1) = 5;
  kx(2) = 10;
  kv(0) = 2.5;
  kv(1) = 2.5;
  kv(2) = 8; 
  
  ros::Subscriber odom_sub = n.subscribe("odom", 10, &odom_callback,
                                         ros::TransportHints().tcpNoDelay());
  ros::Subscriber position_cmd_sub = n.subscribe("position_cmd", 10, &position_cmd_callback,
                                                 ros::TransportHints().tcpNoDelay());

  ros::Subscriber enable_motors_sub = n.subscribe("motors", 2, &enable_motors_callback,
                                                  ros::TransportHints().tcpNoDelay());
  ros::Subscriber corrections_sub = n.subscribe("corrections", 10, &corrections_callback,
                                                ros::TransportHints().tcpNoDelay());

  linear_command_pub = n.advertise<quadrotor_msgs::LinearCommand>("linear_cmd", 10);

  ROS_INFO("Linear Control Begins!");

  ros::spin();

  return 0;
}
