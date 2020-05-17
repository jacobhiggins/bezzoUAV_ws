#include "cmd_romdp.h"

#include <ros/package.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

void joyCb(const sensor_msgs::JoyConstPtr &joy_msgs)
{// left
	if (joy_msgs->buttons[2] == 1)
	{
		attack_cmd.x = 0;
		attack_cmd.y = 0;
		attack_cmd.z = 0;
		// attack
		if (joy_msgs->axes[2] < -0.5)
		{
			attack_flag = true;
			ROS_INFO("Begin Attack on the Left...\n");
			attack_cmd.x = 1;  // attack
			attack_cmd.y = 2;  // y axes
			//attack_cmd.z = -1; // attack type -1

	        attack_cmd.z = 1; // attack type +1
		}
		else
		{
			attack_flag = false;
			ROS_INFO("Remove Attack...\n");

			// remove attack visualization 
			attack_cell.cells.clear();
		}
		attack_pub3.publish(attack_cmd); // sensor 3
                //attack_pub2.publish(attack_cmd); // sensor 2
	}

	// right
	if (joy_msgs->buttons[1] == 1)
	{
		attack_cmd.x = 0;
		attack_cmd.y = 0;
		attack_cmd.z = 0;
		// attack
		if (joy_msgs->axes[2] < -0.5)
		{
			attack_flag = true;
			ROS_INFO("Begin Attack on the right...\n");
			attack_cmd.x = 1;  // attack
			attack_cmd.y = 2;  // y axes
			attack_cmd.z = -1; // attack type -1
                        //attack_cmd.z = 2;  // attack type +2
		}
		else
		{
			attack_flag = false;
			ROS_INFO("Remove Attack...\n");
			
			// remove attack visualization 
			attack_cell.cells.clear();
		}
		attack_pub3.publish(attack_cmd); // sensor 3
                //attack_pub2.publish(attack_cmd); // sensor 2
	}
/*
	if (joy_msgs->axes[2] < -0.5)
	{
		//geometry_msgs::Point attack_cmd;

		// attack
		if (joy_msgs->buttons[2] == 1)
		{
			attack_flag = true;
			ROS_INFO("Begin Attack...\n");
			attack_cmd.x = 1;  // attack
			attack_cmd.y = 2;  // y axes
			//attack_cmd.z = -1; // attack type -1
  
       		        attack_cmd.z = 1; // attack type +1
			attack_pub3.publish(attack_cmd); // sensor 3
		}

		// remove attack
		else if (joy_msgs->buttons[1] == 1)
		{
			attack_flag = false;
			ROS_INFO("Remove Attack...\n");
			attack_cmd.x = 0; 
			attack_cmd.y = 0;
			attack_cmd.z = 0; 
			attack_pub3.publish(attack_cmd); // sensor 3

			// remove attack visualization 
			attack_cell.cells.clear();
		}
	}
*/
}


void test_reason()
{
	if (cur_pos.y >= 0.0) 
	//if (joy_msgs->buttons[2] == 1)
	{
		attack_cmd.x = 0;
		attack_cmd.y = 0;
		attack_cmd.z = 0;
		// attack
		//if (joy_msgs->axes[2] < -0.5)
		if (cur_pos.x <= 8.5)
		{
			attack_flag = true;
			//ROS_INFO("Begin Attack on the Left...\n");
			attack_cmd.x = 1;  // attack
			attack_cmd.y = 2;  // y axes
			//attack_cmd.z = -1; // attack type -1

	        attack_cmd.z = 1; // attack type +1
		}
		else
		{
			attack_flag = false;
			//ROS_INFO("Remove Attack...\n");

			// remove attack visualization 
			attack_cell.cells.clear();
		}
		attack_pub3.publish(attack_cmd); // sensor 3
         //       attack_pub2.publish(attack_cmd); // sensor 2
	}

	// right
	//if (cur_pos.x > 1.5)
	//if (joy_msgs->buttons[1] == 1)
	if (false)
	{
		attack_cmd.x = 0;
		attack_cmd.y = 0;
		attack_cmd.z = 0;
		// attack
		//if (joy_msgs->axes[2] < -0.5)
		if (true)
		{
			attack_flag = true;
			//ROS_INFO("Begin Attack on the right...\n");
			attack_cmd.x = 1;  // attack
			attack_cmd.y = 2;  // y axes
			//attack_cmd.z = -1; // attack type -1
                        attack_cmd.z = 2;  // attack type +2
		}
		else
		{
			attack_flag = false;
			ROS_INFO("Remove Attack...\n");
			
			// remove attack visualization 
			attack_cell.cells.clear();
		}
		attack_pub3.publish(attack_cmd); // sensor 3
                attack_pub2.publish(attack_cmd); // sensor 2
	}
}


// odom Callback function
void odomCb(const nav_msgs::OdometryConstPtr &odom_msgs_)
{
	cur_pos = odom_msgs_->pose.pose.position;
	//std::cout<<cur_pos.x<<std::endl;
	test_reason();
	// visualize attack
	if (attack_flag)
	{
		int ind_x = (int) floor(cur_pos.x + 0.5) + 1;
		int ind_y = (int) floor(cur_pos.y + attack_cmd.z + 0.5) + 1;
		geometry_msgs::Point attack_pos;
		attack_pos.x = ind_x - 1;
		attack_pos.y = ind_y - 1;
		attack_pos.z = 0.01;
		attack_cell.cells.clear();
		attack_cell.cells.push_back(attack_pos);
	}
}

void actionCb(const geometry_msgs::PointConstPtr &action_msgs_)
{
	pos_cmd.position.x = cur_pos.x + action_msgs_->x;
	pos_cmd.position.y = cur_pos.y + action_msgs_->y;
	pos_cmd.position.z = cur_pos.z + action_msgs_->z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_romdp_node");
	ros::NodeHandle nh_;

	ros::Subscriber odom_sub     = nh_.subscribe("odom", 10, &odomCb);
	ros::Subscriber action_sub   = nh_.subscribe("/romdp_node/action", 1, &actionCb);
	ros::Subscriber joy_sub      = nh_.subscribe("joy", 1, &joyCb);
	ros::Publisher pos_pub       = nh_.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 1);
	ros::Publisher tg_pub        = nh_.advertise<geometry_msgs::Point>("target", 1);
	
	ros::Publisher quad_cell_pub   = nh_.advertise<nav_msgs::GridCells>("cmd_romdp_node/quad_cell", 1);
	ros::Publisher obst_cell_pub   = nh_.advertise<nav_msgs::GridCells>("cmd_romdp_node/obst_cell", 1);
	ros::Publisher goal_cell_pub   = nh_.advertise<nav_msgs::GridCells>("cmd_romdp_node/goal_cell", 1);
	ros::Publisher attack_cell_pub   = nh_.advertise<nav_msgs::GridCells>("cmd_romdp_node/attack_cell", 1);
	ros::Publisher vis_pub         = nh_.advertise<visualization_msgs::MarkerArray>( "cmd_romdp_node/obst_marker", 1);


	attack_pub1    = nh_.advertise<geometry_msgs::Point>("cmd_romdp_node/sensor1", 1);
	attack_pub2    = nh_.advertise<geometry_msgs::Point>("cmd_romdp_node/sensor2", 1);
	attack_pub3    = nh_.advertise<geometry_msgs::Point>("cmd_romdp_node/sensor3", 1);

	ros::Publisher run_mdp_pub    = nh_.advertise<std_msgs::Int32>("run_romdp_flag", 1);

	// start position
        std::vector<double> start_position;
	nh_.getParam("start_position", start_position);
        cur_pos.x = start_position[0];
        cur_pos.y = start_position[1];
        cur_pos.z = start_position[2];
	pos_cmd.position = cur_pos;
	//std::cout<<start_position[1]<<std::endl;
/*
	// define control gains
	pos_cmd.kx[0] = 3;
	pos_cmd.kx[1] = 3;
	pos_cmd.kx[2] = 6;
	pos_cmd.kv[0] = 3;
	pos_cmd.kv[1] = 3;
	pos_cmd.kv[2] = 5;
*/
	double time_period = 5.0;
	ros::param::get("cmd_romdp_node/time", time_period);
        ros::param::get("romdp_node/row", row);
        ros::param::get("romdp_node/col", col);

	ros::Duration(5.0).sleep();


	//init environment
	env_init();
	obst_cell_pub.publish(obst_cell);
	goal_cell_pub.publish(goal_cell);
	vis_pub.publish(marker_array);

	// main loop
	int rate = 10;
	ros::Rate loop_rate(rate);
	double start_time = 0.0;
	ros::Time last_time = ros::Time::now();

	// takeoff
	takeoff();

	while (ros::ok())
	{
		check_bound();
		if (start_time > 5.0 && inside_bound)
		{
			// run ARMDP every const time
			if ((ros::Time::now() - last_time).toSec() >= time_period)
			{
				ROS_INFO("Time passed: %f", start_time);
				std_msgs::Int32 run_mdp_cmd;
				run_mdp_cmd.data = 1;
				run_mdp_pub.publish(run_mdp_cmd);
				last_time = ros::Time::now();
			}			
		}

		// publish command
		pos_pub.publish(pos_cmd);
		tg_pub.publish(pos_cmd.position);
		quad_cell_pub.publish(quad_cell);
		attack_cell_pub.publish(attack_cell);

		ros::spinOnce();
		loop_rate.sleep();
		start_time += 1.0/rate;
		//std::cout<<start_time<<std::endl;
	}
}

// check boundary
void check_bound()
{
	int ind_x = (int) floor(cur_pos.x + 0.5) + 1;
	int ind_y = (int) floor(cur_pos.y + 0.5) + 1;

	quad_cell.cells.clear();
	geometry_msgs::Point quad_cell_pos;
	quad_cell_pos.x = ind_x - 1;
	quad_cell_pos.y = ind_y - 1;
	quad_cell_pos.z = 0.01;
	quad_cell.cells.push_back(quad_cell_pos);

	inside_bound = (ind_x < col) && (ind_y <= row) && (ind_y > 0);
}


// takeoff
void takeoff()
{
	//pos_cmd.position.x = cur_pos.x;
	//pos_cmd.position.y = cur_pos.y;
	pos_cmd.position.z = cur_pos.z + 1.0; 
	ROS_INFO("Taking Off...\n");
	//std::cout<<cur_pos.x<<std::endl;
	ros::param::set("/attack_romdp_node/NoiseFlag", true);
}

void land()
{
	pos_cmd.position.x = cur_pos.x;
	pos_cmd.position.y = cur_pos.y;
	pos_cmd.position.z = 0.0; 
	ROS_INFO("Landing...\n");
	ros::param::set("/attack_romdp_node/NoiseFlag", false);
}

// **************************** visualization only ******************************** //
void env_init()
{
	// cells
    attack_cell.header.frame_id =
	obst_cell.header.frame_id = 
	goal_cell.header.frame_id = 
	quad_cell.header.frame_id = "simulator";

	// cell width
	attack_cell.cell_width = obst_cell.cell_width  = goal_cell.cell_width  = quad_cell.cell_width  = 1.0;
	attack_cell.cell_height = obst_cell.cell_height = goal_cell.cell_height = quad_cell.cell_height = 1.0;

	// markers
	visualization_msgs::Marker bound_marker, obst_marker, solid_marker;
	solid_marker.header.frame_id = obst_marker.header.frame_id = bound_marker.header.frame_id = "simulator";
	solid_marker.header.stamp = obst_marker.header.stamp = bound_marker.header.stamp = ros::Time();
	solid_marker.ns = "undeveloped_area";
	obst_marker.ns = "obstacles";
	bound_marker.ns = "boundary";
	bound_marker.id = 100;
	//obst_marker.id = 1;
	//solid_marker.id = 200;

	bound_marker.type = visualization_msgs::Marker::LINE_STRIP;
	obst_marker.type = visualization_msgs::Marker::CYLINDER;
	solid_marker.type = visualization_msgs::Marker::CUBE;
	solid_marker.action = obst_marker.action = bound_marker.action = visualization_msgs::Marker::ADD;
	bound_marker.scale.x = 0.1;
	bound_marker.color.a = 1.0;
	bound_marker.color.r = 1.0;
	bound_marker.color.g = 1.0;
	bound_marker.color.b = 1.0;

	obst_marker.scale.x = 0.7; obst_marker.scale.y = 0.7; obst_marker.scale.z = 1.5;
	obst_marker.color.a = 0.8;
	obst_marker.color.r = 1.0;
	obst_marker.color.g = 0.0;
	obst_marker.color.b = 0.0;

	solid_marker.scale.x = 1; solid_marker.scale.y = 1; solid_marker.scale.z = 1;
	solid_marker.color.a = 0.5;
	solid_marker.color.r = 1.0;
	solid_marker.color.g = 0.0;
	solid_marker.color.b = 1.0;

	// loading position informaiton
    std::ifstream myfile; 
    std::ostringstream ss;
    ss<<row;
    myfile.open((ros::package::getPath("cmd_romdp_pk") + "/env/" + ss.str() +"env.txt").c_str());
    //myfile.open((ros::package::getPath("cmd_romdp_pk") + "/env/5env.txt").c_str());
    string line;
    if (myfile.is_open())
    {
    	while ( !myfile.eof() )
    	{
	    	getline(myfile, line);
			if (line[0] == '#') // comment
			{
				//
				switch (line[1])
				{
					// boundary
					case 'b':
					{
						vector<float> vec;
						float d = 0.0;
						while (getline(myfile, line))
						{
							if (line.length() == 0)
								break;
							stringstream ss(line);
							while (ss >> d)
								vec.push_back(d);
						}
						if ( vec.empty() )
							break;
						// push back the points to marker
						for (int i = 0; i < vec.size(); i+=3)
						{
							geometry_msgs::Point pt;
							pt.x = vec[i];
							pt.y = vec[i+1];
							pt.z = vec[i+2];
							//ROS_INFO("%f", pt.y);
							bound_marker.points.push_back(pt);
						}

						// push back the first point to make the boundary closed
						geometry_msgs::Point pt;
						pt.x = vec[0];
						pt.y = vec[1];
						pt.z = vec[2];
						bound_marker.points.push_back(pt);

						// push back bound_marker
						marker_array.markers.push_back(bound_marker);
						break;
					}

					// obstacles
					case 'o':
					{
						vector<float> vec;
						float d = 0.0;
						while (getline(myfile, line))
						{
							if (line.length() == 0)
								break;
							stringstream ss(line);
							while (ss >> d)
								vec.push_back(d);
						}
						if ( vec.empty() )
							break;
						// push back the points to marker
						for (int i = 0; i < vec.size(); i+=3)
						{
							geometry_msgs::Point pt;
							pt.x = vec[i];
							pt.y = vec[i+1];
							pt.z = vec[i+2];
							obst_cell.cells.push_back(pt);
					   		obst_marker.pose.position = pt;
   							obst_marker.pose.position.z = 0.75;
   							// different namespace and id will create different markers
   							stringstream num2str;
   							num2str << i;
   							//obst_marker.ns = "obst" + num2str.str();
   							//ROS_INFO("%s", num2str.str().c_str());
   							obst_marker.id = i;

							marker_array.markers.push_back(obst_marker);
						}
						break;

					}

					// goals
					case 'g':
					{
						vector<float> vec;
						float d = 0.0;
						while (getline(myfile, line))
						{
							if (line.length() == 0)
								break;
							stringstream ss(line);
							while (ss >> d)
								vec.push_back(d);
						}

						if ( vec.empty() )
							break;
						// push back the points to marker
						for (int i = 0; i < vec.size(); i+=3)
						{
							geometry_msgs::Point pt;
							pt.x = vec[i];
							pt.y = vec[i+1];
							pt.z = vec[i+2];
							goal_cell.cells.push_back(pt);
						}
						break;
					}

					// undeveloped area
					case 'u':
					{
						vector<float> vec;
						float d = 0.0;
						while (getline(myfile, line))
						{
							if (line.length() == 0)
								break;

							stringstream ss(line);
							while (ss >> d)
								vec.push_back(d);
						}

						if ( vec.empty() )
							break;
						// push back the points to marker
						double pos_x = 0, pos_y = 0, pos_z = 0;
						int count = 0;
						for (int i = 0; i < vec.size(); i+=3)
						{
							pos_x += vec[i];
							pos_y += vec[i+1];
							pos_z += vec[i+2];
							count++;
							/*
							geometry_msgs::Point pt;
							pt.x = vec[i];
							pt.y = vec[i+1];
							pt.z = vec[i+2];
							solid_marker.pose.position = pt;
							*/
						}
						solid_marker.pose.position.x = pos_x / count;
						solid_marker.pose.position.y = pos_y / count;
						solid_marker.pose.position.z = 0.3;
						solid_marker.scale.x = fabs(pos_x / count - vec[0]) * 2 - 0.05;
						solid_marker.scale.y = fabs(pos_y / count - vec[1]) * 2 - 0.05;
						solid_marker.scale.z = 0.6;
						// different id creates different marker
						solid_marker.id = 10;
						marker_array.markers.push_back(solid_marker);
						break;
					}

					default:
						break;
				}
			}
		}

    }
    else
    {
    	std::cout<<"Failed to open file: environment" <<std::endl;
    	exit(-1);
    }
    myfile.close();
/*
	// set the boundary of map
	visualization_msgs::Marker marker;
	marker.header.frame_id = "simulator";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	geometry_msgs::Point s1, s2, s3, s4;
	s1.x = s4.x = -0.5;
	s2.x = s3.x = col - 0.5;
	s1.y = s2.y = -0.5;
	s3.y = s4.y = row - 0.5;
	marker.points.push_back(s1);
	marker.points.push_back(s2);
	marker.points.push_back(s3);
	marker.points.push_back(s4);
	marker.points.push_back(s1);

	marker_array.markers.push_back(marker);

    // frame_id
    attack_cell.header.frame_id =
	obst_cell.header.frame_id = 
	goal_cell.header.frame_id = 
	quad_cell.header.frame_id = "simulator";

	// cell width
	attack_cell.cell_width = obst_cell.cell_width  = goal_cell.cell_width  = quad_cell.cell_width  = 1.0;
	attack_cell.cell_height = obst_cell.cell_height = goal_cell.cell_height = quad_cell.cell_height = 1.0;

	// loading position informaiton
    std::ifstream myFile; 
    std::ostringstream ss;ss<<row;
    myFile.open((ros::package::getPath("cmd_romdp_pk") + "/env/" + ss.str() +"env.txt").c_str());
    int count = 0;
    double buffer[100];

    if (myFile.is_open())
    {
    	while(!myFile.eof())
    	{
    		myFile >> buffer[count++];
    	}

    }
    else
    {
    	std::cout<<"Failed to open file: environment" <<std::endl;
    	exit(-1);
    }
    myFile.close();

    // read obstacles
 	int obst_ind = 1;
    while (obst_ind <= buffer[0] * 3)
	{
		geometry_msgs::Point pos;
		pos.x = buffer[obst_ind++];
		pos.y = buffer[obst_ind++];
		pos.z = buffer[obst_ind++];
		obst_cell.cells.push_back(pos);



		// create marker
		visualization_msgs::Marker marker;
        marker.header.frame_id = "simulator";
   		marker.header.stamp = ros::Time();
   		marker.ns = "my_namespace";
   		marker.id = obst_ind;
   		marker.type = visualization_msgs::Marker::CYLINDER;
   		marker.action = visualization_msgs::Marker::ADD;
   		marker.pose.position = pos;
   		marker.pose.position.z = 0.75;
   		/*
  		marker.pose.orientation.x = 0.0;
  		marker.pose.orientation.y = 0.0;
  		marker.pose.orientation.z = 0.0;
  		marker.pose.orientation.w = 1.0;
		
  		marker.scale.x = 0.7;
  		marker.scale.y = 0.7;
  		marker.scale.z = 1.5;
  		marker.color.a = 0.8;
  		marker.color.r = 1.0;
  		marker.color.g = 0.0;
  		marker.color.b = 0.0;

		marker_array.markers.push_back(marker);
	}
	// read goals
 	int goal_ind = obst_ind + 1;
    while (goal_ind <= obst_ind + buffer[obst_ind] * 3)
	{
		geometry_msgs::Point pos;
		pos.x = buffer[goal_ind++];
		pos.y = buffer[goal_ind++];
		pos.z = buffer[goal_ind++];
		goal_cell.cells.push_back(pos);
	}

*/
}
