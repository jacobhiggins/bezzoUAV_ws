#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/SO3Command.h>
#include <iris_simulator_pk/iris_simulator.h>

typedef struct _ControlInput
{
  double rpm[4];
} ControlInput;


geometry_msgs::Twist mpc_cmd;
static Eigen::Vector3d ext_force, ext_moment;
void stateToOdomMsg(const IRISSimulator::Quadrotor::State &state, nav_msgs::Odometry &odom);
void quadToImuMsg(const IRISSimulator::Quadrotor &quad, sensor_msgs::Imu &imu);

static ControlInput getMPCControl(const IRISSimulator::Quadrotor &quad, const geometry_msgs::Twist &mpc_cmd)
{
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  // const double kf = _kf - mpc_cmd.kf_correction;
  const double kf = _kf; // kf_correction always seems to = 0
  const double km = _km/_kf*kf;
  const Eigen::Matrix<double, 4, 4> coeff = quad.getTorqueArmLength();
  const double m = quad.getMass();
  const double g = quad.getGravity();
  double force, M1, M2, M3;
  force = mpc_cmd.linear.z + m*g;
  M1 = mpc_cmd.angular.x;
  M2 = mpc_cmd.angular.y;
  M3 = mpc_cmd.angular.z;

  Eigen::Matrix<double, 4, 1> B_vec, w_sq_vec;
  B_vec << force/kf, M1/kf, M2/kf, M3/km;
  w_sq_vec = coeff.inverse() *  B_vec;
  float w_sq[4];
  w_sq[0] = w_sq_vec(0);
  w_sq[1] = w_sq_vec(1);
  w_sq[2] = w_sq_vec(2);
  w_sq[3] = w_sq_vec(3);

  ControlInput control;
  for(int i = 0; i < 4; i++)
  {
    if(w_sq[i] < 0)
      w_sq[i] = 0;

    control.rpm[i] = sqrtf(w_sq[i]);
  }
  return control;
}

static void mpc_cmd_cb(const geometry_msgs::Twist &msg)
{
  mpc_cmd.linear.z = msg.linear.z;
  mpc_cmd.angular.x = msg.angular.x;
  mpc_cmd.angular.y = msg.angular.y;
  mpc_cmd.angular.z = msg.angular.z;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "iris_simulator_so3");

  ros::NodeHandle n("~");

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_raw", 100);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  ros::Subscriber mpc_cmd_sub = n.subscribe("/mpc_control/mpc_cmd",100,&mpc_cmd_cb,ros::TransportHints().tcpNoDelay());

  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);
  const ros::Duration odom_pub_duration(1/odom_rate);

  std::string quad_name;
  n.param("quadrotor_name", quad_name, std::string("quadrotor"));

  IRISSimulator::Quadrotor quad;
  IRISSimulator::Quadrotor::State state = quad.getState();

  // Initial Position
  std::vector<double> start_position;
  start_position.assign(3, 0.0);
  ros::param::get("start_position", start_position);

  state.x(0) = start_position[0];
  state.x(1) = start_position[1];
  state.x(2) = start_position[2];
  quad.setState(state);
  //std::cout<<state.x(1)<<std::endl;

  ros::Rate r(simulation_rate);
  const double simulation_dt = 1/simulation_rate;

  ControlInput control;

  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  odom_msg.header.frame_id = "/odom";
  odom_msg.child_frame_id = "/" + quad_name;
  imu_msg.header.frame_id = "/" + quad_name;

  ros::Time next_odom_pub_time = ros::Time::now();
  while(n.ok())
  {
    // ROS_INFO("MPC Simulator");
    ros::spinOnce();

    /*
    control = getControl(quad, command);
    //quad.setExternalForce(ext_force);
    */ // Original Control
    control = getMPCControl(quad, mpc_cmd);
    quad.setExternalMoment(ext_moment);
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
    quad.step(simulation_dt);

    ros::Time tnow = ros::Time::now();

    if(tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      state = quad.getState();
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad, imu_msg);
      odom_msg.header.stamp = tnow;
      imu_msg.header.stamp = tnow;
      odom_pub.publish(odom_msg);
      imu_pub.publish(imu_msg);
    }

    r.sleep();
  }

  return 0;
}

void stateToOdomMsg(const IRISSimulator::Quadrotor::State &state, nav_msgs::Odometry &odom)
{
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

void quadToImuMsg(const IRISSimulator::Quadrotor &quad, sensor_msgs::Imu &imu)
{
  const IRISSimulator::Quadrotor::State state = quad.getState();
  Eigen::Quaterniond q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  const double kf = quad.getPropellerThrustCoefficient();
  const double m = quad.getMass();
  const Eigen::Vector3d &external_force = quad.getExternalForce();
  const double g = quad.getGravity();
  const double thrust = kf*state.motor_rpm.square().sum();
  Eigen::Vector3d acc;
  if(state.x(2) < 1e-4)
  {
    acc = state.R*(external_force/m + Eigen::Vector3d(0,0,g));
  }
  else
  {
    acc = thrust/m*Eigen::Vector3d(0,0,1) + state.R.inverse()*external_force/m;
  }

  imu.linear_acceleration.x = acc(0);
  imu.linear_acceleration.y = acc(1);
  imu.linear_acceleration.z = acc(2);
}
