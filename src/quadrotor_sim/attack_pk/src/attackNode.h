#ifndef __ATTACKNODE_H
#define __ATTACKNODE_H

#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <eigen3/Eigen/Dense>

class AttackNode
{
  private:
    // subscriber
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;

    // publisher
    ros::Publisher state_pub;
    ros::Publisher sensor1_pub;
    ros::Publisher sensor2_pub;
    ros::Publisher sensor3_pub;

    ros::NodeHandle nh_;

    // state variable
    nav_msgs::Odometry state_final, s1, s2, s3;

    bool AF, NF;
    int AT;
    float timeElapsed;

  public:
    AttackNode();
    ~AttackNode();
  

    // callback function
    void odomCb(const nav_msgs::Odometry odomPtr);
    void imuCb(const sensor_msgs::ImuConstPtr &imuPtr);
  
    // attack function
    void add_attack(const nav_msgs::Odometry odomPtr);
    void calculate_odom();
    void add_noise(const nav_msgs::Odometry odomPtr);


    
    // main loop
    void Loop();
    void publish_odom();
    

};

//
geometry_msgs::Point pos(const geometry_msgs::Pose pos_msg);
geometry_msgs::Quaternion quat(const geometry_msgs::Pose quat_msg);
geometry_msgs::Vector3 vel(const geometry_msgs::Twist vel_msg);
geometry_msgs::Vector3 ang(const geometry_msgs::Twist ang_msg);

#endif
