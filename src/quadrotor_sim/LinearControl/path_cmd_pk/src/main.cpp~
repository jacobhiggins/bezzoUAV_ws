#include "ros/ros.h"
#include "drone_pos_pub.h"


static PosCommand pos_command;
static nav_msgs::Odometry odom_msg;
static int count = 0;
static bool is_posCb_flag;
static bool isGoalReached;

// define waypts variable
// rectangle
const double waypts_1[4][3] = {{-2, 0, 1}, {-2, -2, 1}, {0, -2, 1}, {0, 0, 1}};
// house
const double waypts_2[10][3] = {{-1, 0, 1}, {1, 0, 1}, {-1, -2, 1}, {1, -2, 1},
                                {0, -3, 1}, {-1,-2,1}, {-1,  0, 1}, {1, -2, 1},
                                {1, 0,  1}, {0, 0, 1}};


// odometry callback function
void odomCb(const nav_msgs::OdometryConstPtr &odomPtr)
{
  // get odometry message
  odom_msg.pose = odomPtr->pose;

  if (isGoalReached)
  {
    return;
  }
  
  if (is_posCb_flag)
  {
    // get target dist:
    double diff_0 = pos_cmd.position.x - odomPtr->pose.pose.position.x;
    double diff_1 = pos_cmd.position.y - odomPtr->pose.pose.position.y;
    double diff_2 = pos_cmd.position.z - odomPtr->pose.pose.position.z;

    double diffDist = diff_0 * diff_0 + diff_1 * diff_1 + diff_2 * diff_2; 

    if (diffDist < threshold) 
    {
      // reach the target and then hover for 1 seconds
      double lastTime = ros::Time::now().toSec();
      while ((double) ros::Time::now().toSec() - lastTime < 1.0) 
      {
        cmd_pub.publish(pos_cmd);
      }
      setNewTarget();
    }
  }
}

// takeoff function
void PosCommand::takeoffCb(const std_msgs::Empty emp_msg)
{
  pos_cmd.position.x = odom_msg.pose.pose.position.x;
  pos_cmd.position.y = odom_msg.pose.pose.position.y;
  pos_cmd.position.z = 1.0;
  ROS_INFO("Taking Off!");
  isTookoff = true;
  ros::param::set("/drone_attack/NF", true);
}

// land function
void PosCommand::landCb(const std_msgs::Empty emp_msg)
{
  if (!isTookoff) 
  {
     ROS_INFO("The Quadrotor has not TOOK OFF!!!");
     return;
  }
  ros::param::set("/drone_attack/NF", false);
  pos_cmd.position.x = odom_msg.pose.pose.position.x;
  pos_cmd.position.y = odom_msg.pose.pose.position.y;
  pos_cmd.position.z = 0.0;
  
  ROS_INFO("Landing");
  isTookoff = false;
  resetEverything();
}

void PosCommand::resetEverything()
{
  threshold = 0.05;
  count = 0;
  is_posCb_flag = false;
  isGoalReached = false;
}


// position send callback function
void PosCommand::posCb(const std_msgs::StringConstPtr &traj_msg)
{
  resetEverything();
  if (!isTookoff) 
  {
     ROS_INFO("Take off first!");
     return;
  }
  pos_cmd.velocity.x = 0;
  pos_cmd.velocity.y = 0;
  pos_cmd.velocity.z = 0;
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

void PosCommand::setNewTarget()
{
  if (count > max_size)
  {
    isGoalReached = true;
    ROS_INFO("Goal Reached!");
    return;
  };
  // set way points
  pos_cmd.position.x = waypts[count][0];
  pos_cmd.position.y = waypts[count][1];
  pos_cmd.position.z = waypts[count][2];
  count++;
  ROS_INFO("Target: x:%f, y: %f, z: %f\n ", pos_cmd.position.x, pos_cmd.position.y, pos_cmd.position.z);

}


void PosCommand::velCb(const geometry_msgs::Vector3ConstPtr &vec)
{
  if (!isTookoff) 
  {
     ROS_INFO("The Quadrotor has not TOOK OFF!!!");
     return;
  }
  pos_cmd.position = odom_msg.pose.pose.position;
  pos_cmd.velocity.x = vec->x;
  pos_cmd.velocity.y = vec->y;
  pos_cmd.velocity.z = vec->z;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_pos_pub");

  ros::NodeHandle nh_;

  ros::Publisher cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 10);
  ros::Publisher tg_pub = nh_.advertise<geometry_msgs::Point>("target", 10);  

  ros::Subscriber odom_sub = nh_.subscribe("odom", 10, &PosCommand::odomCb);;
  ros::Subscriber takeoff_sub = nh_.subscribe("takeoff", 10, &PosCommand::takeoffCb);
  ros::Subscriber land_sub = = nh_.subscribe("land", 10, &PosCommand::landCb);;
  ros::Subscriber pos_sub = nh_.subscribe("pos", 10, &PosCommand::posCb);;
  ros::Subscriber vel_sub = nh_.subscribe("vel", 10, &PosCommand::velCb);;
  
  ROS_INFO("Running!");
  
  resetEverything();

  ros::spin();

  return 0;
}
