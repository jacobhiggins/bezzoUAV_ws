#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

#include <quadrotor_msgs/PositionCommand.h>

#include <nav_msgs/GridCells.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


nav_msgs::GridCells quad_cell, goal_cell, obst_cell, attack_cell;

visualization_msgs::MarkerArray marker_array;

geometry_msgs::Point cur_pos, attack_cmd;
quadrotor_msgs::PositionCommand pos_cmd;
bool inside_bound = true;
bool attack_flag = false;
int row, col;

ros::Publisher attack_pub1, attack_pub2, attack_pub3;

void takeoff();
void land();
void check_bound();
void env_init();

/* statement
geometry::Point attack_cmd;

attack_cmd.x --> attack flag    (0, 1)    ---> (remove, attack)
attack_cmd.y --> dimension      (1, 2, 3) ---> (x, y, z);
attack_cmd.z --> attack type    (-1, 1)   ---> bias attack
*/
