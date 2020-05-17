#ifndef __LINEAR_CONTROL_H__
#define __LINEAR_CONTROL_H__

#include <eigen3/Eigen/Geometry>

class LinearControl
{
 private:
  // Inputs for the controller
  double mass_;
  double g_;
  double maxAng;
  // Inertia Matrix
  double Ixx, Ixy, Ixz;
  double Iyx, Iyy, Iyz;
  double Izx, Izy, Izz;
  
  Eigen::Matrix<double, 12, 1> X_vec, X_0;
  Eigen::Matrix<double, 4,1> u_vec;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d euler_, omega_;
  
  // LQR variable
  Eigen::Matrix<double, 4, 12> K;
  Eigen::Matrix<double, 12, 12> A;
  Eigen::Matrix<double, 12, 4> B;
  Eigen::Matrix<double, 4, 4> R;
  Eigen::Matrix<double, 12, 12> Q;

  // Outputs of the controller
  Eigen::Vector3d force_;
  Eigen::Vector3d moment_;
  Eigen::Quaterniond orientation_;

 public:
  LinearControl();
  ~LinearControl();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d &position);
  void setVelocity(const Eigen::Vector3d &velocity);
  void setEuler(const Eigen::Vector3d &euler);
  void setAngleVel(const Eigen::Vector3d &omega);

  void calculateControl(const Eigen::Vector3d &des_pos,
                        const Eigen::Vector3d &des_vel,
                        const Eigen::Vector3d &des_acc,
                        const double des_yaw,
                        const double des_yaw_dot,
                        const Eigen::Vector3d &kx,
                        const Eigen::Vector3d &kv,
                        const Eigen::Vector3d &kR,
                        const Eigen::Vector3d &kOm);

  const Eigen::Vector3d getComputedForce(void);
  const Eigen::Vector3d getComputedMoment(void);
  const Eigen::Quaterniond &getComputedOrientation(void);

  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif
