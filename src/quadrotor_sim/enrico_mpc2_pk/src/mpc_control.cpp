#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
//#include <enrico_mpc_pk/MPCControl.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <error.h>
#include <iostream>
// #include "common.h"
#include "/home/bezzo/ExplicitMPC/glpk-uav/common.h" /* Message structure defined here*/

// #define MPC_CTRL "/home/bezzo/bezzoUAV_ws/src/quadrotor_sim/enrico_mpc2_pk/src/mpc_ctrl"
#define MPC_CTRL "/home/bezzo/ExplicitMPC/glpk-uav/mpc_ctrl"
#define PRINT_ERROR(x) fprintf(stderr, "%s:%i: %s , errno= %i \n", __FILE__, __LINE__, x,errno);

//static MPCControl controller;
static geometry_msgs::Twist trpy_cmd; // Control inputs (trpy) from MPC to simulator
static ros::Publisher trpy_cmd_pub; // Publisher for trpy_cmd
static ros::Publisher state_pub; // Publisher of current state information, for debugging
static Eigen::Vector3d des_pos, des_rpy, des_vel, des_pqr;
static double current_yaw = 0;
double state[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double ref[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double input[4] = {0,0,0,0};
struct msg_to_mpc msg_sent;
struct msg_from_mpc msg_recv;

int from_mpc[2];
int to_mpc[2];

static void publishTRPY(void)
{
    trpy_cmd.linear.z = msg_recv.input[0]; // thrust
    trpy_cmd.angular.x = msg_recv.input[1]; // roll
    trpy_cmd.angular.y = msg_recv.input[2]; // pitch
    trpy_cmd.angular.z = msg_recv.input[3]; // yaw
    trpy_cmd_pub.publish(trpy_cmd);
}

static void position_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &cmd){
    des_pos = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);

    ref[0] = des_pos[0];
    ref[1] = des_pos[1];
    ref[2] = des_pos[2];
    ref[3] = 0;
    ref[4] = 0;
    ref[5] = 0;
    ref[6] = 0;
    ref[7] = 0;
    ref[8] = 0;
    ref[9] = 0;
    ref[10] = 0;
    ref[11] = 0;

    // ROS_INFO("Inside position_cmd callback");
    // ROS_INFO("Commanded Position: (%f,%f,%f)",x,y,z);
    // std::cout << "X position: " << x;
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
{

  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);
  const Eigen::Vector3d pqr(odom->twist.twist.angular.x,
                            odom->twist.twist.angular.y,
                            odom->twist.twist.angular.z);
  const tf::Quaternion q(odom->pose.pose.orientation.x,
                        odom->pose.pose.orientation.y,
                        odom->pose.pose.orientation.z,
                        odom->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);

  msg_sent.state[0] = position[0] - ref[0];
  msg_sent.state[1] = position[1] - ref[1];
  msg_sent.state[2] = position[2] - ref[2];
  msg_sent.state[3] = r - ref[3];
  msg_sent.state[4] = p - ref[4];
  msg_sent.state[5] = y - ref[5];
  msg_sent.state[6] = velocity[0] - ref[6];
  msg_sent.state[7] = velocity[1] - ref[7];
  msg_sent.state[8] = velocity[2] - ref[8];
  msg_sent.state[9] = pqr[0] - ref[9];
  msg_sent.state[10] = pqr[1] - ref[10];
  msg_sent.state[11] = pqr[2] - ref[11];

  geometry_msgs::Twist state_msg;
  state_msg.linear.x = position[0];
  state_msg.linear.y = position[1];
  state_msg.linear.z = position[2];
  state_msg.angular.x = r;
  state_msg.angular.y = p;
  state_msg.angular.z = y;

//   ROS_INFO("RPY: (%f,%f,%f)",r,p,y);
  state_pub.publish(state_msg);

}

int main(int argc, char **argv){

    // ********** Setup MPC **********
    // Setting up pipe to enrico solver
    pipe(to_mpc);
    pipe(from_mpc);

    /* Strings of read/write file descriptors */
	char fd_rd[5];
	char fd_wr[5];
	/* Parameters to be passed to MPC_CTRL */
	char * args[] = {MPC_CTRL, fd_rd, fd_wr, "/home/bezzo/ExplicitMPC/glpk-uav/json/uav12_iris.json", NULL};
    pid_t pid;


    if ( !(pid = fork()) ) {
		/* CHILD CODE ONLY */
		sprintf(fd_rd, "%i", to_mpc[0]);   /* storing read end */
		close(to_mpc[1]);                  /* closing write end */
		sprintf(fd_wr, "%i", from_mpc[1]); /* storing write end */
		close(from_mpc[0]);                /* closing read end */

		/* Now jumping to the child code */
        /* remeber to specify the pat of the executable. TODO get absolute path*/
		execve(MPC_CTRL, args, NULL);
		PRINT_ERROR("Error in execve\n");
	}


    // ********** ROS **********

    ros::init(argc, argv, "mpc_control");

    ros::NodeHandle n("~");

    trpy_cmd_pub = n.advertise<geometry_msgs::Twist>("mpc_cmd",10);
    state_pub = n.advertise<geometry_msgs::Twist>("state",10);

    ros::Rate rate(10);

    ros::Subscriber position_cmd_sub = n.subscribe("/iris_position_cmd", 10, &position_cmd_cb,
                                                 ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_sub = n.subscribe("/iris_odom", 10, &odom_cb,
                                         ros::TransportHints().tcpNoDelay());

    while(ros::ok()){
        
        ros::spinOnce();
        /* Filling the message */
        msg_sent.sender = getpid();
        clock_gettime(CLOCK_MONOTONIC, &(msg_sent.timestamp));

        if (write(to_mpc[1], &msg_sent, sizeof(msg_sent)) == -1)
            ROS_INFO("Issue in write");
    
            // READ IS BLOCKING
        if (read(from_mpc[0], &msg_recv, sizeof(msg_recv)) == -1){
            PRINT_ERROR("Issue in write");
            //return;
        }
        // printf("Got control action %f\n", msg_recv.input[0]);
        publishTRPY();


         rate.sleep();
    }

    // std::string quadrotor_name;
    // n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
    // so3_command.header.frame_id = "/" + quadrotor_name;
}