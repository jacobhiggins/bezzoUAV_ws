#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

class FakeOdom{

private:
    ros::Publisher pub;
    ros::NodeHandle nh_;

public:
    FakeOdom(){
        pub = nh_.advertise<nav_msgs::Odometry>("odom_raw", 100);
    }
  
    void loop(){
        ros::Rate pub_rate(100);
  
	while(nh_.ok()){
	    ros::spinOnce();

	    nav_msgs::Odometry msg;

	    msg.pose.pose.position.x = 5;
	    msg.pose.pose.position.y = 5;
	    msg.pose.pose.position.z = 5;

	    pub.publish(msg);
	    
	    // sleep
	    pub_rate.sleep();
	}
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "fo");

    FakeOdom fo;

    fo.loop();

    return 0;

}
