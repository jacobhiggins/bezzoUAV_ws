#include "model.h"
#include "PolicyIteration.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

// global variables
static std::string pk_path;


ros::Publisher action_pub;
ros::Subscriber odom_sub, sensor1_sub, sensor2_sub, sensor3_sub, run_mdp_sub;

geometry_msgs::Point cur_pos;

double sensor_data[3][2];

bool run_mdp_flag = false;



// functions
void read_transition_matrix(const int N_, const int M_, Eigen::MatrixXd *trans_3d_);
void read_reward_matrix(const int N_, const int M_, Eigen::MatrixXd &reward_);

void take_action(const int dim_1_, const int dim_2_, const Eigen::MatrixXd policy);


void check_sensor_measurement(const int row, const int col, int *index);
void check_bound(const int row, const int col);

/*
void run_ardmp(const int N_, const int M_, 
	           const int c1, const int c2, 
	           const int num_c1, const int num_c2, 
	           Eigen::MatrixXd *trans_3d_);
*/
