#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sched.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
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
#include <fstream>
#include "../../../../mpc/mpc_interface.h"

#define PRINT_ERROR(x) fprintf(stderr, "%s:%i: %s , errno= %i \n", __FILE__, __LINE__, x,errno);

/* Uncomment below if needed to trace it */
//#define TRACE_ME

// Very important to keep!
#define USE_PROCESS_PINNING
#define CPU_ID_PARENT 1 // CPU where parent process lives

#ifdef TRACE_ME
#ifdef ROS_INFO
#undef ROS_INFO
#endif
#define ROS_INFO(...)
#endif /* TRACE_ME */


#define STRLEN_COMMAND 100

/* GLOBAL VARIABLES */
double max_time = 0;
int must_exit = 0;

//static MPCControl controller;
std::string sim_type;
static bool debug = true;
static std::ofstream debugfile;
static geometry_msgs::Twist trpy_cmd; // Control inputs (trpy) from MPC to simulator
static nav_msgs::Odometry mpc_state; // State associated with control inputs
static ros::Publisher trpy_cmd_pub; // Publisher for trpy_cmd
static ros::Publisher mpc_state_pub; // Publisher for state with mpc
static ros::Publisher state_pub; // Publisher of current state information, for debugging
static ros::Subscriber position_cmd_sub;
static ros::Subscriber odom_sub;
static Eigen::Vector3d des_pos, des_rpy, des_vel, des_pqr;
static double current_yaw = 0;

/* Pointer to shared memory used to communicate between ROS and MPC */
struct shared_data * msg;

double state[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double ref[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double input[4] = {0,0,0,0};


void term_handler(int signum);

static void publishTRPY(void)
{
	trpy_cmd.linear.z = msg->input[0]; // thrust
	trpy_cmd.angular.x = msg->input[1]; // roll
	trpy_cmd.angular.y = msg->input[2]; // pitch
	trpy_cmd.angular.z = msg->input[3]; // yaw
	
	mpc_state.pose.pose.position.x = msg->state[0] + ref[0]; // x
	mpc_state.pose.pose.position.y = msg->state[1] + ref[1]; // y
	mpc_state.pose.pose.position.z = msg->state[2] + ref[2]; // z
	mpc_state.pose.pose.orientation.x = msg->state[3]; // roll
	mpc_state.pose.pose.orientation.y = msg->state[4]; // pitch
	mpc_state.pose.pose.orientation.z = msg->state[5]; // yaw
	mpc_state.twist.twist.linear.x = msg->state[6]; // x dot
	mpc_state.twist.twist.linear.y = msg->state[7]; // y dot
	mpc_state.twist.twist.linear.z = msg->state[8]; // z dot
	mpc_state.twist.twist.angular.x = msg->state[9]; // roll dot
	mpc_state.twist.twist.angular.y = msg->state[10]; // pitch dot
	mpc_state.twist.twist.angular.z = msg->state[11]; // yaw dot
	mpc_state.pose.covariance[0] = msg->input[0];
	mpc_state.pose.covariance[1] = msg->input[1];
	mpc_state.pose.covariance[2] = msg->input[2];
	mpc_state.pose.covariance[3] = msg->input[3];
	
	trpy_cmd_pub.publish(trpy_cmd);
	mpc_state_pub.publish(mpc_state);
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

static void position_Matlab_cmd_cb(const geometry_msgs::Twist::ConstPtr &cmd)
{
	ref[0] = cmd->linear.x;
	ref[1] = cmd->linear.y;
	ref[2] = cmd->linear.z;
	ref[3] = 0;
	ref[4] = 0;
	ref[5] = 0;
	ref[6] = 0;
	ref[7] = 0;
	ref[8] = 0;
	ref[9] = 0;
	ref[10] = 0;
	ref[11] = 0;
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

  msg->state[0] = position[0] - ref[0];
  msg->state[1] = position[1] - ref[1];
  msg->state[2] = position[2] - ref[2];
  msg->state[3] = r - ref[3];
  msg->state[4] = p - ref[4];
  msg->state[5] = y - ref[5];
  msg->state[6] = velocity[0] - ref[6];
  msg->state[7] = velocity[1] - ref[7];
  msg->state[8] = velocity[2] - ref[8];
  msg->state[9] = pqr[0] - ref[9];
  msg->state[10] = pqr[1] - ref[10];
  msg->state[11] = pqr[2] - ref[11];

  geometry_msgs::Twist state_msg;
  state_msg.linear.x = position[0] - ref[0];
  state_msg.linear.y = position[1] - ref[1];
  state_msg.linear.z = position[2] - ref[2];
  state_msg.angular.x = r - ref[3];
  state_msg.angular.y = p - ref[4];
  state_msg.angular.z = y - ref[5];

//   ROS_INFO("RPY: (%f,%f,%f)",r,p,y);
  state_pub.publish(state_msg);

}

static void odom_matlab_cb(const nav_msgs::Odometry::ConstPtr &odom)
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
  const Eigen::Vector3d rpy(odom->pose.pose.orientation.x,
                        odom->pose.pose.orientation.y,
                        odom->pose.pose.orientation.z);

  msg->state[0] = position[0] - ref[0];
  msg->state[1] = position[1] - ref[1];
  msg->state[2] = position[2] - ref[2];
  msg->state[3] = rpy[0] - ref[3];
  msg->state[4] = rpy[1] - ref[4];
  msg->state[5] = rpy[2] - ref[5];
  msg->state[6] = velocity[0] - ref[6];
  msg->state[7] = velocity[1] - ref[7];
  msg->state[8] = velocity[2] - ref[8];
  msg->state[9] = pqr[0] - ref[9];
  msg->state[10] = pqr[1] - ref[10];
  msg->state[11] = pqr[2] - ref[11];

  geometry_msgs::Twist state_msg;
  state_msg.linear.x = position[0] - ref[0];
  state_msg.linear.y = position[1] - ref[1];
  state_msg.linear.z = position[2] - ref[2];
  state_msg.angular.x = rpy[0] - ref[3];
  state_msg.angular.y = rpy[1] - ref[4];
  state_msg.angular.z = rpy[2] - ref[5];

//   ROS_INFO("RPY: (%f,%f,%f)",r,p,y);
  state_pub.publish(state_msg);

}

static void write_debug(ros::Duration t){
	std::string time = std::to_string(t.toSec());
	std::string state = std::to_string(msg->state[0]) + ","
		+ std::to_string(msg->state[1]) + ","
		+ std::to_string(msg->state[2]) + ","
		+ std::to_string(msg->state[3]) + ","
		+ std::to_string(msg->state[4]) + ","
		+ std::to_string(msg->state[5]) + ","
		+ std::to_string(msg->state[6]) + ","
		+ std::to_string(msg->state[7]) + ","
		+ std::to_string(msg->state[8]) + ","
		+ std::to_string(msg->state[9]) + ","
		+ std::to_string(msg->state[10]) + ","
		+ std::to_string(msg->state[11]);
	
	std::string input = std::to_string(msg->input[0]) + ","
		+ std::to_string(msg->input[1]) + ","
		+ std::to_string(msg->input[2]) + ","
		+ std::to_string(msg->input[3]);
	
	debugfile << time << ",";
	debugfile << state << ",";
	debugfile << input << "\n";
	debugfile.flush();
}


/*
 * Set prio priority (high number => high priority) and pin the
 * invoking process to CPU cpu_id
 */
void sched_set_prio_affinity(uint32_t prio, int cpu_id);


int main(int argc, char **argv){
	int shm_id;
	
#ifdef USE_PROCESS_PINNING
	sched_set_prio_affinity(
		sched_get_priority_max(SCHED_FIFO),
		CPU_ID_PARENT);
#endif
	
	// ********** Debug ********
	if (debug)
		debugfile.open("debug_file.csv");
	
	// ********** ROS **********
	
	//    ros::init(argc, argv, "mpc_control");
	ros::init(argc, argv, "mpc_control",ros::init_options::NoSigintHandler);
	
	/* Setting up the signal handler for termination */
	struct sigaction sa;
	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	/* Getting the shared memory area */
	shm_id = shmget(MPC_SHM_KEY, 0, 0);
	if (shm_id == -1) {
		PRINT_ERROR("shmget failed");
	}
	msg = (struct shared_data *)shmat(shm_id, NULL, 0);
	
	ros::NodeHandle n("~");

	trpy_cmd_pub = n.advertise<geometry_msgs::Twist>("mpc_cmd",1);
	mpc_state_pub = n.advertise<nav_msgs::Odometry>("mpc_state",1);
	state_pub = n.advertise<geometry_msgs::Twist>("state",1);
	
	/*
	 * FIXME: better if this number is a configuration constant on
	 * top such as
	 *
	 * #define RATE 100
	 *
	 * and then replace with
	 *
	 * ros::Rate rate(RATE);
	 */
	ros::Rate rate(100);
	
	n.getParam("param",sim_type);
	ROS_INFO("sim type: %s",sim_type.c_str());
	
	if(sim_type.compare("matlab")==0) {
		position_cmd_sub = n.subscribe("/matlab_position_cmd",
					       10,&position_Matlab_cmd_cb,
					       ros::TransportHints().tcpNoDelay());
		odom_sub= n.subscribe("/iris_matlab_odom",
				      10,&odom_matlab_cb,
				      ros::TransportHints().tcpNoDelay());
	} else if(sim_type.compare("ros")==0) {
		position_cmd_sub = n.subscribe("/iris_position_cmd",
					       10, &position_cmd_cb,
					       ros::TransportHints().tcpNoDelay());
		odom_sub = n.subscribe("/iris_odom_raw", 10, &odom_cb,
				       ros::TransportHints().tcpNoDelay());
	} else if(sim_type.compare("")==0) {
		std::string message;
		message = std::string("No simulation type given. Please use rosrun with _param:=<sim_type>.\n");
		message = message + std::string("Valid sim types are \"matlab\" or \"ros\".");
		ROS_WARN("%s",message.c_str());
		std::exit(1);
	} else {
		ROS_WARN("Please use parameter \"matlab\" or \"ros\" to indicate simulation.");
		std::exit(1);
	}

	ros::Time start = ros::Time::now();
	ros::Duration dt;
	
	while(ros::ok()){
		ros::spinOnce();
		/*
		 * Now asking MPC to compute the optimal input for
		 * us. The state was written in the shared memory
		 * already by  publishTRPY();
		 */
		sem_post(msg->sems+MPC_SEM_STATE_WRITTEN);
		/*
		 * And now waiting for the optimal input to be
		 * written
		 */
		sem_wait(msg->sems+MPC_SEM_INPUT_WRITTEN);
		if (must_exit) {
			shmdt(msg);
			break;
		}
		
		// printf("Got control action %f\n", msg->input[0]);
		publishTRPY();
		// if (debug){
		// 	write_debug(t);
		// }
		
		rate.sleep();
	}
	
	// std::string quadrotor_name;
	// n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
	// so3_command.header.frame_id = "/" + quadrotor_name;
}

void term_handler(int signum)
{
	ROS_INFO("Process %d: got signal %d", getpid(), signum);
	must_exit = 1;
}


void sched_set_prio_affinity(uint32_t prio, int cpu_id)
{
	cpu_set_t  mask;

	/* Set CPU affinity */
	CPU_ZERO(&mask);
	CPU_SET(cpu_id, &mask);
	if (sched_setaffinity(0, sizeof(mask), &mask) != 0) {
		PRINT_ERROR("sched_setaffinity");
		exit(-1);
	}

	/* Set priority */
#if SCHED_SETATTR_IN_SCHED_H
	/* EB: TO BE TESTED */
	struct sched_attr attr;
	
	bzero(&attr, sizeof(attr));
	attr.size = sizeof(attr);
	attr.sched_policy = SCHED_FIFO;
	attr.sched_priority = prio;
	if (sched_setattr(0, &attr, 0) != 0) {
		PRINT_ERROR("sched_setattr");
		exit(-1);
	}
#else
	char launched[STRLEN_COMMAND];  /* String with launched command */

	snprintf(launched, STRLEN_COMMAND,
		 "sudo chrt -f -p %d %d", prio, getpid());
	system(launched);

	/* EB: next code fails because it needs sudo */
	/*
	struct sched_param param;
	
	bzero(&param, sizeof(param));
	param.sched_priority = prio;
	if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
		PRINT_ERROR("sched_setscheduler");
		exit(-1);
	}
	*/
#endif
}
