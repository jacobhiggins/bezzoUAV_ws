#ifndef __IRIS_SIMULATOR_QUADROTOR_H__
#define __IRIS_SIMULATOR_QUADROTOR_H__

#include <boost/array.hpp>
#include <eigen3/Eigen/Core>

namespace IRISSimulator
{

class Quadrotor
{
 public:
  struct State
  {
    Eigen::Vector3d x;
    Eigen::Vector3d v;
    Eigen::Matrix3d R;
    Eigen::Vector3d omega;
    Eigen::Array4d motor_rpm;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  Quadrotor();

  const Quadrotor::State &getState(void) const;
  void setState(const Quadrotor::State &state);

  double getMass(void) const;
  void setMass(double mass);

  double getGravity(void) const;
  void setGravity(double g);

  const Eigen::Matrix3d &getInertia(void) const;
  void setInertia(const Eigen::Matrix3d &inertia);

  double getArmLength(void) const;
  void setArmLength(double d);

  const Eigen::Matrix<double, 4,4> &getTorqueArmLength(void) const;

  double getPropRadius(void) const;
  void setPropRadius(double r);

  double getPropellerThrustCoefficient(void) const;
  void setPropellerThrustCoefficient(double kf);

  double getPropellerMomentCoefficient(void) const;
  void setPropellerMomentCoefficient(double km);

  double getMotorTimeConstant(void) const;
  void setMotorTimeConstant(double k);

  const Eigen::Vector3d &getExternalForce(void) const;
  void setExternalForce(const Eigen::Vector3d &force);

  const Eigen::Vector3d &getExternalMoment(void) const;
  void setExternalMoment(const Eigen::Vector3d &moment);

  double getMaxRPM(void) const;
  void setMaxRPM(double max_rpm);

  double getMinRPM(void) const;
  void setMinRPM(double min_rpm);

  // Inputs are desired RPM for the motors
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
  void setInput(double u1, double u2, double u3, double u4);

  // Runs the actual dynamics simulation with a time step of dt
  void step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 22> InternalState;
  void operator()(const Quadrotor::InternalState &x, Quadrotor::InternalState &dxdt, const double /* t */);

 private:
  void updateInternalState(void);

  double g_; // gravity
  double mass_;
  Eigen::Matrix3d J_; // Inertia
  double kf_;
  double km_;
  double prop_radius_;
  double arm_length_;
  double arm0_x, arm0_y, arm1_x, arm1_y, arm2_x, arm2_y, arm3_x, arm3_y;
  Eigen::Matrix<double, 4, 4> coeff_;
  double motor_time_constant_; // unit: sec
  double max_rpm_;
  double min_rpm_;
  Quadrotor::State state_;
  Eigen::Array4d input_;
  Eigen::Vector3d external_force_;
  Eigen::Vector3d external_moment_;
  Eigen::Matrix<double, 4, 2> l_mat_;

  InternalState internal_state_;
};

}
#endif
