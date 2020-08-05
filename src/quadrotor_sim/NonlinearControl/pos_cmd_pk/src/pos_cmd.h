#include "ros/ros.h"
#include "std_msgs/String.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include <string>


ros::Publisher cmd_pub;
ros::Publisher tg_pub;


// position command 
quadrotor_msgs::PositionCommand pos_cmd;

// odom variable
nav_msgs::Odometry odom_msg;


// parameters
float threshold;
int count;
int max_size;
bool is_posCb_flag;
bool isGoalReached;
bool isTookoff;
double waypts[16][3];

void setNewTarget();
void resetEverything();
		


// define waypts variable
// rectangle
const double waypts_1[12][3] = {{2, 0, 1}, {2, 2, 1}, {0, 2, 1}, {0, 0, 1},
                                {4, 0, 2}, {4, 4, 2}, {0, 4, 2}, {0, 0, 2},
                                {8, 0, 4}, {8, 8, 4}, {0, 8, 4}, {0, 0, 4}};

// house
const double waypts_2[10][3] = {{-1, 0, 1}, {1, 0, 1}, {-1, 2, 1}, {1, 2, 1},
                                {0, 3, 1}, {-1,2,1}, {-1,  0, 1}, {1, 2, 1},
                                {1, 0,  1}, {0, 0, 1}};

const double waypts_3[31][3] = {{0,2,1},
                                {0,4,1},
                                {0,6,1},
                                {0,8,1},
                                {0,10,1}, //
                                {0,12,1},
                                {0,14,1},
                                {0,16,1},
                                {0,15,1},
                                {0,14,1}, //
                                {0,13,1},
                                {0,12,1},
                                {0,11,1},
                                {0,10,1},
                                {0,9,1}, //
                                {0,8,1},
                                {0,7,1},
                                {0,6,1},
                                {0,5,1},
                                {0,4,1}, //
                                {0,3,1},
                                {0,2,1},
                                {0,1,1},
                                {0,0,1},
                                {0,16,1}, //
                                {0,0,1},
                                {0,15,1},
                                {0,2,0},
                                {0,14,1},
                                {0,3,0}, //
                                {0,0,1}};