#include "nav_msgs/Odometry.h"
#include "ros/ros.h"



nav_msgs::Odometry ground_truth, s1, s2, s3;
bool AttackFlag, NoiseFlag;
double timeElapsed;

geometry_msgs::Point attack_sensor1, attack_sensor2, attack_sensor3;

void add_noise();
nav_msgs::Odometry add_attack(const int AttackAxes, const int AttackType);