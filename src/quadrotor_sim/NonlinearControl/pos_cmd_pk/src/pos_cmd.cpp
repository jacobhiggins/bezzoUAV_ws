#include "pos_cmd.h"

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
	  // Helps stabilize at the waypoint
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
void takeoffCb(const std_msgs::Empty emp_msg)
{
  pos_cmd.position.x = odom_msg.pose.pose.position.x;
  pos_cmd.position.y = odom_msg.pose.pose.position.y;
  pos_cmd.position.z = 1.0;
  ROS_INFO("Taking Off!");
  isTookoff = true;
  ros::param::set("attack_node/NoiseFlag", true);
}

// land function
void landCb(const std_msgs::Empty emp_msg)
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
  
  ROS_INFO("Landing");
  isTookoff = false;
  resetEverything();
}

// position send callback function
void posCb(const std_msgs::StringConstPtr &traj_msg)
{
  resetEverything();
  if (!isTookoff) 
  {
     ROS_INFO("Take off first!");
     return;
  }
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
    for (int i = 0; i < 31; i++)
    {
      for (int j = 0; j < 3; j ++)
        waypts[i][j] = waypts_3[i][j];
    }
    max_size = 31;
  }
  
  setNewTarget();
  // flag
  is_posCb_flag = true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_cmd_node");
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub = nh_.subscribe("odom_raw", 10, &odomCb);;
  ros::Subscriber takeoff_sub = nh_.subscribe("takeoff", 10, &takeoffCb);
  ros::Subscriber land_sub = nh_.subscribe("land", 10, &landCb);
  ros::Subscriber pos_sub = nh_.subscribe("pos", 10, &posCb);

  cmd_pub     = nh_.advertise<quadrotor_msgs::PositionCommand>(ros::this_node::getNamespace()+"/position_cmd", 10); 
  tg_pub      = nh_.advertise<geometry_msgs::Point>("target", 10); 

/*
  pos_cmd.kx[0] = 3;
  pos_cmd.kx[1] = 3;
  pos_cmd.kx[2] = 6;
  pos_cmd.kv[0] = 3;
  pos_cmd.kv[1] = 3;
  pos_cmd.kv[2] = 5;

  // define control gains
  ros::param::get("~gains/pos/x", pos_cmd.kx[0]);
  ros::param::get("~gains/pos/y", pos_cmd.kx[1]);
  ros::param::get("~gains/pos/z", pos_cmd.kx[2]);
  ros::param::get("~gains/vel/x", pos_cmd.kv[0]);
  ros::param::get("~gains/vel/y", pos_cmd.kv[1]);
  ros::param::get("~gains/vel/z", pos_cmd.kv[2]);
  */
  resetEverything();
  
  ros::Rate pub_rate(10);

  while (nh_.ok())
  {
    cmd_pub.publish(pos_cmd);
    tg_pub.publish(pos_cmd.position);

    ros::spinOnce();
    pub_rate.sleep();
  }

  return 0;
}



void resetEverything()
{
  threshold = 0.1;
  count = 0;
  is_posCb_flag = false;
  isGoalReached = false;
}

void setNewTarget()
{
  ROS_INFO("%d",count);
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
