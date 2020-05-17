#include "ros/ros.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "nav_msgs/Odometry.h"
#include "path_cmd.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

static PosCommand pos_command;
static ros::Publisher cmd_pub, tg_pub;

static quadrotor_msgs::PositionCommand pos_cmd;
static nav_msgs::Odometry odom_msg;


static bool isGoalReached, isTookoff, is_posCb_flag;

static int count = 0;
static int max_size;

static geometry_msgs::Point start_pt, stop_pt;

double start_time;
double last_time;
double threshold;

// define waypts variable
// rectangle
const double waypts_1[4][3] = {{2, 0, 1}, {2, 2, 1}, {0, 2, 1}, {0, 0, 1}};
// house
const double waypts_2[10][3] = {{-1, 0, 1}, {1, 0, 1}, {-1, 2, 1}, {1, 2, 1},
                                {0, 3, 1}, {-1,2,1}, {-1,  0, 1}, {1, 2, 1},
                                {1, 0,  1}, {0, 0, 1}};
double waypts[10][3];

// function statement
void setNewTarget();
void resetEverything();


// odometry callback function
void odomCb(const nav_msgs::OdometryConstPtr &odomPtr)
{
  // get odometry message
  odom_msg.pose = odomPtr->pose;
  //if (isGoalReached)
  //{
  //  return;
  //}
  if (is_posCb_flag)
  {
    // get target dist:
    double diff_0 = stop_pt.x - odomPtr->pose.pose.position.x;
    double diff_1 = stop_pt.y - odomPtr->pose.pose.position.y;
    double diff_2 = stop_pt.z - odomPtr->pose.pose.position.z;

    double diffDist = diff_0 * diff_0 + diff_1 * diff_1 + diff_2 * diff_2; 

    if (diffDist < threshold * threshold)
    //if (false) 
    {
      //pos_cmd.position = stop_pt;
      setNewTarget();
      // reach the target and then hover for 1 seconds
      //double lastTime = ros::Time::now().toSec();
      //while ((double) ros::Time::now().toSec() - lastTime < 1.0) 
      //{
      //  cmd_pub.publish(pos_cmd);
      //}
    }
    else 
    {
      if (ros::Time::now().toSec() - last_time >= 0.05)
      {
        last_time = ros::Time::now().toSec();
        double dt = last_time - start_time;
        Eigen::Matrix3d pos_mat = pos_command.getPath(dt);
        pos_cmd.position.x = pos_mat(0,0);
        pos_cmd.position.y = pos_mat(0,1);
        pos_cmd.position.z = pos_mat(0,2);
        pos_cmd.velocity.x = pos_mat(1,0);
        pos_cmd.velocity.y = pos_mat(1,1);
        pos_cmd.velocity.z = pos_mat(1,2);
        pos_cmd.acceleration.x = pos_mat(2,0);
        pos_cmd.acceleration.y = pos_mat(2,1);
        pos_cmd.acceleration.z = pos_mat(2,2);
      }
    }
  }

  cmd_pub.publish(pos_cmd);
  tg_pub.publish(stop_pt); 
}

// takeoff function
void takeoffCb(const std_msgs::Empty &emp_msg)
{
  pos_cmd.position.x = odom_msg.pose.pose.position.x;
  pos_cmd.position.y = odom_msg.pose.pose.position.y;
  pos_cmd.position.z = 1.0;
  stop_pt = pos_cmd.position;
  ROS_INFO("Taking Off!");
  isTookoff = true;
  //ros::param::set("/attack_node/NoiseFlag", true);
}

// land function
void landCb(const std_msgs::Empty &emp_msg)
{
  if (!isTookoff) 
  {
     ROS_INFO("The Quadrotor has not TOOK OFF!!!");
     return;
  }
  ros::param::set("/attack_node/NoiseFlag", false);
  pos_cmd.position.x = odom_msg.pose.pose.position.x;
  pos_cmd.position.y = odom_msg.pose.pose.position.y;
  pos_cmd.position.z = 0.0;
  stop_pt = pos_cmd.position;
  ROS_INFO("Landing");
  isTookoff = false;
  resetEverything();
}

// position send callback function
void posCb(const std_msgs::StringConstPtr &traj_msg)
{
  if (!isTookoff) 
  {
     ROS_INFO("Take off first!");
     return;
  }
  resetEverything();
  // select trajectory data
  if (traj_msg->data.length() == 5)
  {
    ROS_INFO("Flying Trajectory:  house ");
    for (int i = 0; i < 10; i++)
    {
      for (int j = 0; j < 3; j ++)
        waypts[i][j] = waypts_2[i][j];
    }
    max_size = 9;
  }
  else
  {
    ROS_INFO("Flying Trajectory:  rectangle ");
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 3; j ++)
        waypts[i][j] = waypts_1[i][j];
    }
    max_size = 3;
  }
  
  setNewTarget();
  // flag
  is_posCb_flag = true;
}

void setNewTarget()
{
  if (count > max_size)
  {
    isGoalReached = true;
    //ROS_INFO("Goal Reached!");
    return;
  };
  // set way points
  start_pt = stop_pt;
  stop_pt.x = waypts[count][0];
  stop_pt.y = waypts[count][1];
  stop_pt.z = waypts[count][2];
  count++;
  ROS_INFO("Target: x:%f, y: %f, z: %f\n ", stop_pt.x, stop_pt.y, stop_pt.z);
  pos_command.calPath(Eigen::Vector3d(start_pt.x, start_pt.y, start_pt.z), 
                      Eigen::Vector3d(stop_pt.x, stop_pt.y, stop_pt.z));
  last_time = start_time = ros::Time::now().toSec();
}

void resetEverything()
{
  threshold = 0.02;
  count = 0;
  is_posCb_flag = false;
  isGoalReached = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_cmd_node");

  ros::NodeHandle nh_;

  cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 1);
  tg_pub = nh_.advertise<geometry_msgs::Point>("target", 1);  

  ros::Subscriber odom_sub = nh_.subscribe("odom", 10, &odomCb);
  ros::Subscriber takeoff_sub = nh_.subscribe("takeoff", 1, &takeoffCb);
  ros::Subscriber land_sub = nh_.subscribe("land", 1, &landCb);
  ros::Subscriber pos_sub = nh_.subscribe("pos", 10, &posCb);
 /*
  // define control gains
  pos_cmd.kx[0] = 5;
  pos_cmd.kx[1] = 5;
  pos_cmd.kx[2] = 10;
  pos_cmd.kv[0] = 2.5;
  pos_cmd.kv[1] = 2.5;
  pos_cmd.kv[2] = 8;  
  //         kp_z: 50
  //        kd_z: 10
  //       kp_xy: 5
  //       kd_xy: 2.5000

  //pos_cmd.kx[0] = 6;
  //pos_cmd.kx[1] = 6;
  //pos_cmd.kx[2] = 10;
  //pos_cmd.kv[0] = 3;
  //pos_cmd.kv[1] = 3;
  //pos_cmd.kv[2] = 5; 
*/

  resetEverything();

  ros::spin();

  return 0;
}
