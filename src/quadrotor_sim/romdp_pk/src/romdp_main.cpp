#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>

#include "header/newType.h"
#include "model.h"
#include "PolicyIteration.h"
#include "romdp.h"

// loading transition matrix
void read_transition_matrix(const int N_, const int M_, Eigen::MatrixXd *trans_3d_)
{
	std::ifstream myFile;
	myFile.open((pk_path + "transition.txt").c_str());
	int count = 0;
	double mat_buffer[M_*N_*N_];

	if (myFile.is_open())
	{
		while(!myFile.eof())
			myFile >> mat_buffer[count++];
	}
	else
	{
		std::cout<<"Error Open transition matrix file!"<<std::endl;
		exit(-1);
	} 
	myFile.close();

	for (int act = 0; act < M_; act++)
		for (int i = 0; i < N_; i++)
			for (int j = 0; j < N_; j++)
				trans_3d_[act](i, j) = mat_buffer[act * N_ * N_ + i * N_ + j];

}

// loading reward matrix
void read_reward_matrix(const int N_, const int M_, Eigen::MatrixXd &reward_)
{
	std::ifstream myFile;
	myFile.open((pk_path + "reward.txt").c_str());
	int count = 0;
	double mat_buffer[M_*N_];

	if (myFile.is_open())
	{
		while(!myFile.eof())
			myFile >> mat_buffer[count++];
	}
	else
	{
		std::cout<<"Error Open reward matrix file!"<<std::endl;
		exit(-1);
	} 
	myFile.close();

	for (int i = 0; i < N_; i++)
		for (int act = 0; act < M_; act ++)
			reward_(i, act) = mat_buffer[i*M_ + act];
	//std::cout<<reward_<<std::endl;
}

// odom Callback function
void odomCb(const nav_msgs::OdometryConstPtr &odom_msgs_)
{
	cur_pos = odom_msgs_->pose.pose.position;
	//std::cout<<cur_pos.x<<std::endl;
}

// sensor 1 callback function
void sensor1_Cb(const nav_msgs::OdometryConstPtr &s1_msgs_)
{
	sensor_data[0][0] = s1_msgs_->pose.pose.position.x;
	sensor_data[0][1] = s1_msgs_->pose.pose.position.y;
	//cur_pos = odom_msgs_->pose.pose.position;
}

// sensor 2 callback function
void sensor2_Cb(const nav_msgs::OdometryConstPtr &s2_msgs_)
{
	sensor_data[1][0] = s2_msgs_->pose.pose.position.x;
	sensor_data[1][1] = s2_msgs_->pose.pose.position.y;
	//cur_pos = odom_msgs_->pose.pose.position;
}

// sensor 3 callback function
void sensor3_Cb(const nav_msgs::OdometryConstPtr &s3_msgs_)
{
	sensor_data[2][0] = s3_msgs_->pose.pose.position.x;
	sensor_data[2][1] = s3_msgs_->pose.pose.position.y;

	//cur_pos = s3_msgs_->pose.pose.position;
}

void run_mdp_Cb(const std_msgs::Int32ConstPtr data_)
{
	run_mdp_flag = (data_->data == 1);
}

// main function
int main(int argc, char **argv)
{
	// ros
	ros::init(argc, argv, "romdp_node");
	ros::NodeHandle nh_;

	action_pub = nh_.advertise<geometry_msgs::Point>("romdp_node/action", 1);
	odom_sub  = nh_.subscribe("odom", 10, &odomCb);
	sensor1_sub  = nh_.subscribe("attack_romdp_node/odom_sensor1", 10, &sensor1_Cb);
	sensor2_sub  = nh_.subscribe("attack_romdp_node/odom_sensor2", 10, &sensor2_Cb);
	sensor3_sub  = nh_.subscribe("attack_romdp_node/odom_sensor3", 10, &sensor3_Cb);
    run_mdp_sub  = nh_.subscribe("run_romdp_flag", 1, &run_mdp_Cb);

    //********************************* initialize MDP parameters ***************************//
    int dim_1 = 5;  // row
    int dim_2 = 5;  // col
    std::string matrix_path = "5cells";
    ros::param::get("romdp_node/row", dim_1);
    ros::param::get("romdp_node/col", dim_2);
    ros::param::get("romdp_node/matrix_path", matrix_path);

    pk_path = ros::package::getPath("romdp_pk") + "/matrix/" + matrix_path + "/";

	int N = dim_1 * dim_2;
	int M = 4;
	double discount = 0.9;
	int eval_type = 0;
	int max_iter = 1000;

	// Model
	MDP::Model model;
	model.set_state_num(N);
	model.set_action_num(M);
	model.setDiscount(discount);

	// create transition matrix and reward matrix
	// reward function
	Eigen::MatrixXd reward(N, M);
	// transition matrix
	Eigen::MatrixXd *trans_3d = new Eigen::MatrixXd[M];
	Eigen::MatrixXd *trans_3d_use = new Eigen::MatrixXd[M];
	for (int i = 0; i < M; i++)
	{
		trans_3d[i] = Eigen::MatrixXd::Zero(N, N);
		trans_3d_use[i] = Eigen::MatrixXd::Zero(N, N);
	}
	
	read_transition_matrix(N, M, trans_3d);
	read_reward_matrix(N, M, reward);


	// Policy Iteration
	MDP::PolicyIteration policy_iteration(model);
    //policy_iteration.set_eval_type(eval_type);  // default
	//policy_iteration.set_max_iter(max_iter); // default

    // ************************* END *********************************************************//

	// wait other ros node to be ready
	ros::Duration(5.0).sleep();
	int rate = 10;
	ros::Rate loop_rate(rate);
        ros::Time tic;

    // main loop
	while (ros::ok())
	{

		Policy_and_Value pv;
		//if (start_time > 5.0 && inside_flag)
		if (run_mdp_flag)
		{
			tic = ros::Time::now();
			int ind[4];
			check_sensor_measurement(dim_1, dim_2, ind);
                        // reset MDP 
			policy_iteration.reset();
                        // set reward matrix
			policy_iteration.set_reward_matrix(reward); 
			// set transition matrix
			for (int act = 0; act < M; act++)
				for (int i = 0; i < N; i++)
					for (int j = 0; j < N; j++)
						trans_3d_use[act](i, j) = trans_3d[act](i,j);
			policy_iteration.set_transition_matrix(trans_3d_use);  

			//std::cout<<ind[0]<<", "<<ind[1]<<", "<<ind[2]<<", "<<ind[3]<<std::endl;
	
                        // attacked, update transition matirx and reward matrix
			if (ind[3] == 1) 
			{ 
				ROS_INFO("Detected Attacking...");
				policy_iteration.update_transition_matrix(ind[0], ind[1], ind[2]);
				policy_iteration.update_reward_matrix(ind[0], ind[1], ind[2]);
				ROS_INFO("Sensor Measurements: %i, %i, %i", ind[0] + 1, ind[1] + 1, ind[2] + 1);
			}

			//ros::Time tic = ros::Time::now();
			//std::cout<<"Loop begins..."<<std::endl;

			// calculate policy
			ROS_INFO("Run ROMDP...");
			//std::cout<<"Loop begins..."<<std::endl;
			pv = policy_iteration.eval();
			//std::cout<<"Loop ends..."<<std::endl;

			//int iter_loop = policy_iteration.get_iter();

			// delete transition matrix to free memory
			//delete [] trans_3d;

			// print out policy and value
			//std::cout<<"iter: \n"<<iter_loop<<std::endl;
			//std::cout<<"value: \n"<<pv.value<<std::endl;
			//std::cout<<"policy: \n"<<pv.policy<<std::endl;

			// take action
			ROS_INFO("Running time: %f seconds.", (ros::Time::now() - tic).toSec());
			take_action(dim_1, dim_2, pv.policy);
			run_mdp_flag = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// check sensor output
void check_sensor_measurement(const int row, const int col, int *index)
{
	for (int i = 0; i < 3; i++)
	{
		index[i] = ((int) floor(sensor_data[i][0] + 0.5) + 
			        (int) floor(sensor_data[i][1] + 0.5) * col);   
	}

	// check if being attacked
	index[3] = (index[0] == index[1]) && (index[1] == index[2]) ? 0 : 1;

	//************************
	//if ((index[0] <= 25) && (index[1] <= 25 ) && (index[2] <= 25))
	//	index[3] = (index[0] == index[1]) && (index[1] == index[2]) ? 0 : 1;
	//else
	//	index[3] = 0;


	//************************
}

// choose action with current policy
void take_action(const int dim_1_, const int dim_2_, const Eigen::MatrixXd policy_)
{
	int count = 0;
	Eigen::MatrixXd c_mat(dim_1_, dim_2_);
	for (int i = 0; i < dim_1_; i++)
		for (int j = 0; j< dim_2_; j++)
			c_mat(i, j) = policy_(count++);
	//std::cout<<c_mat<<std::endl;
	ROS_INFO("Position: ROW: %d, COL: %d", (int) floor(cur_pos.y + 0.5) + 1, (int) floor(cur_pos.x + 0.5) + 1);
	int dir = c_mat((int) floor(cur_pos.y + 0.5), (int) floor(cur_pos.x + 0.5));
	int mx = 0, my = 0;
	ROS_INFO("Taking action...");
	switch (dir)
	{
		case 1: // moving north
		{
			mx = 0;
			my = 1;
			ROS_INFO("Moving to North...");
			break;
		}
		case 2: // moving south
		{
			mx = 0;
			my = -1;
			ROS_INFO("Moving to South...");
			break; 
		}
		case 3:  // moving west
		{
			mx = -1;
			my = 0;
			ROS_INFO("Moving to West...");
			break;
		}
		case 4:  // moving east
		{
			mx = 1;
			my = 0;
			ROS_INFO("Moving to East...");
			break;
		}
	}
	std::cout<<"\n"<<std::endl;
	// publish actions
	geometry_msgs::Point action_;
	action_.x = mx;
	action_.y = my;
	action_pub.publish(action_);
}


