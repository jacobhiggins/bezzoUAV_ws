#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <eigen3/Eigen/Dense>


nav_msgs::Odometry kf_odom;


// altitude
double alt;

// IMU data
Eigen::Vector3d gyro, acc, mag;

