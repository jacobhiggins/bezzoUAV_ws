
#include "galois_kalman_filter.h"

void imuCallback(const sensor_msgs::ImuConstPtr imu_msgs_)
{

  kf_odom.header.frame_id = imu_msgs_->header.frame_id;

  // angular velocity
  gyro(0) = imu_msgs_->angular_velocity.x;
  gyro(1) = imu_msgs_->angular_velocity.y;
  gyro(2) = imu_msgs_->angular_velocity.z;

  // acceleration
  acc(0) = imu_msgs_->linear_acceleration.x;
  acc(1) = imu_msgs_->linear_acceleration.y;
  acc(2) = imu_msgs_->linear_acceleration.z;

  // magnetomoter
  // convert quaternion to angles
  float q0, q1, q2, q3;
  q0 = imu_msgs_->orientation.w;
  q1 = imu_msgs_->orientation.x;
  q2 = imu_msgs_->orientation.y;
  q3 = imu_msgs_->orientation.z;
  //mag(2) = -atan2(-2*(q1*q2 - q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3);
  //mag(1) = 
  //mag(0) = 

}

void odomCallbck(const nav_msgs::OdometryConstPtr odom_msgs_)
{
  // height
  alt = odom_msgs_->pose.pose.position.z;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "galois_kf_node");

  ros::NodeHandle nh_;

  ros::Publisher  odom_pub = nh_.advertise<nav_msgs::Odometry>("kf_estimated_odom", 1); 
  ros::Subscriber imu_sub  = nh_.subscribe<sensor_msgs::Imu>("imu", 1, &imuCallback);
  ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &odomCallbck);

  /******************************************/
  // kalman filter initial

  ros::Time last_time = ros::Time::now();


  /******************************************/
 // main loop
  ros::Rate loop_rate(100);
  while (ros::ok())
  {

    kf_odom.header.stamp = ros::Time::now();


    /******************************************/
    // kalman filter predict
    double dt = (ros::Time::now() - last_time).toSec();



    /******************************************/


    // publish esitmated odom
    odom_pub.publish(kf_odom);

    last_time = ros::Time::now();

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;

}