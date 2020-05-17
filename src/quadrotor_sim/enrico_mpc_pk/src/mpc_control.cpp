#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <enrico_mpc_pk/MPCControl.h>
#include <tf/transform_datatypes.h>
#include "common.h" /* Message structure defined here*/

#define MPC_CTRL "mpc_ctrl"
#define PRINT_ERROR(x) fprintf(stderr, "%s:%i: %s , errno= %i \n", __FILE__, __LINE__, x,errno);

//static MPCControl controller;
static geometry_msgs::Twist trpy_cmd; // Control inputs (trpy) from MPC to simulator
static ros::Publisher trpy_cmd_pub; // Publisher for trpy_cmd
static Eigen::Vector3d des_pos;

int from_mpc[2];
int to_mpc[2];

static void publishTRPY(void)
{
    trpy_cmd.linear.x = 3.14;
    trpy_cmd_pub.publish(trpy_cmd);
}

static void position_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &cmd){
    des_pos = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    
    ROS_DEBUG("Inside position_cmd callback");
}

int main(int argc, char **argv){

    ros::init(argc, argv, "mpc_control");

    ros::NodeHandle n("~");

    trpy_cmd_pub = n.advertise<geometry_msgs::Twist>("mpc_cmd",10);

    ros::Rate rate(1);

    ros::Subscriber position_cmd_sub = n.subscribe("position_cmd", 10, &position_cmd_cb,
                                                 ros::TransportHints().tcpNoDelay());

    while(ros::ok()){
        publishTRPY();
        ros::spinOnce();
         rate.sleep();
    }

    // std::string quadrotor_name;
    // n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
    // so3_command.header.frame_id = "/" + quadrotor_name;
}