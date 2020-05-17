#include "Linear_Control.h"
#include <iostream>
#include <eigen3/Eigen/Geometry>

LinearControl::LinearControl()
{
  mass_ = 1.282;
  g_ = 9.81; 
  //maxAng = 0.26; //15 degree
  maxAng = 0.4;

  // Inertia Matrix
/*double Ixx = 8.7952e-3,   Ixy = 6.39935e-4,     Ixz = 2.5927e-5;
  double Iyx = 6.39935e-4,   Iyy = 5.14714e-3,     Iyz = 7.203e-6;
  double Izx = 2.5927e-5, Izy = 7.203e-6,     Izz = 1.3624726e-2;
*/
  Ixx = 8.7952e-3;      Ixy = 6.39935e-4;    Ixz = 0;
  Iyx = 6.39935e-4;     Iyy = 5.14714e-3;    Iyz = 0;
  Izx = 0;              Izy = 0;             Izz = 1.3624726e-2;
/*
  // set Matrix A
  A = Eigen::Matrix<double, 12, 12>::Zero();
  A(0,6) = A(1,7) = A(2,8) = A(3,9) = A(4,10) = A(5,11) = 1;

  // set Matrix B
  B = Eigen::Matrix<double, 12, 4>::Zero();
  B(8,0) = 1.0 / mass_;
  B(11,3) = 1.0 / Ixx;
  B(9,1) = - Iyy / (Ixy * Ixy - Ixx * Iyy);
  B(9,2) =  Ixy / (Ixy * Ixy - Ixx * Iyy);
  B(10,1) = Ixy / (Ixy * Ixy - Ixx * Iyy);
  B(10,2) = - Ixx / (Ixy * Ixy - Ixx * Iyy);

  // set Matrix Q
  Q = 10 * Eigen::Matrix<double, 12, 12>::Identity();

  // set Matrix R
  R = Eigen::Matrix<double, 4, 4>::Identity();
*/
  // K Matrix
  K << 0.0,     0.0,     173.2051,  0.0,      0.0,       0.0,      0.0,       0.0,        21.0736,    0.0,      0.0,     0.0, 
       0.0,    -100.0,   0.0,       50.7836,  0.2564,    0.0,      0.0818,    -32.1766,   0.0,        0.9444,   0.0408,  0.0,
       100.0,   0.0,     0.0,       0.2564,   49.3217,   0.0,      31.7101,   -0.0818,    0.0,        0.0408,   0.7116,  0.0,
       0.0,     0.0,     0.0,       0.0,      0.0,       44.7214,  0.0,       0.0,        0.0,        0.0,      0.0,     1.1040;
}

LinearControl::~LinearControl()
{

}

void LinearControl::setMass(const double mass)
{
  mass_ = mass;
}

void LinearControl::setGravity(const double g)
{
  g_ = g;
}

void LinearControl::setPosition(const Eigen::Vector3d &position)
{
  pos_ = position;
}

void LinearControl::setVelocity(const Eigen::Vector3d &velocity)
{
  vel_ = velocity;
}

void LinearControl::setEuler(const Eigen::Vector3d &euler)
{
  euler_ = euler;
}

void LinearControl::setAngleVel(const Eigen::Vector3d &omega)
{
  omega_ = omega;
}
void LinearControl::calculateControl(const Eigen::Vector3d &des_pos,
                                  const Eigen::Vector3d &des_vel,
                                  const Eigen::Vector3d &des_acc,
                                  const double des_yaw,
                                  const double des_yaw_dot,
                                  const Eigen::Vector3d &kx,
                                  const Eigen::Vector3d &kv,
                                  const Eigen::Vector3d &kR,
                                  const Eigen::Vector3d &kOm)
{
  //*************************** Linear Controller **********************************//
 
  Eigen::Vector3d acc_d = des_acc + kx.asDiagonal() * (des_pos - pos_)
                                  + kv.asDiagonal() * (des_vel - vel_);
  force_ = mass_ * (Eigen::Vector3d(0,0,g_) + acc_d);

  Eigen::Vector3d des_euler, des_omega;
  des_euler(0) = 1.0/g_ * (acc_d(0) * sin(des_yaw) - acc_d(1) * cos(des_yaw));
  des_euler(1) = 1.0/g_ * (acc_d(0) * cos(des_yaw) + acc_d(1) * sin(des_yaw));
  des_euler(2) = des_yaw;

  for (int i = 0; i<3; i++)
  {
    if (des_euler(i) > maxAng)
    { 
      des_euler(i) = maxAng;
    }
    else if (des_euler(i) < -maxAng)
    {
      des_euler(i) = -maxAng;
    } 
  }
  
  des_omega(2) = des_yaw_dot;
//  moment_ = kR.asDiagonal() * (des_euler - euler_) + kOm.asDiagonal() * (des_omega - omega_);

  //******************************** END *******************************************//



  //*************************** LQR Controller ************************************//

  /*
  A(6,3) = g_ * sin(des_yaw);
  A(6,4) = g_ * cos(des_yaw);
  A(7,3) = -g_ * cos(des_yaw);
  A(7,4) = g_ * sin(des_yaw);
  */
/*
  Eigen::Vector3d des_euler;
  des_euler(1) = 1.0/g_ * (des_acc(0) * cos(des_yaw) + des_acc(1) * sin(des_yaw));
  des_euler(0) = 1.0/g_ * (des_acc(0) * sin(des_yaw) - des_acc(1) * cos(des_yaw));
  des_euler(2) = des_yaw;

  X_vec << des_pos, des_euler, des_vel, 0, 0, des_yaw_dot;
  X_0 << pos_, euler_, vel_, omega_;
  u_vec = K * (X_vec - X_0);

  force_ = u_vec(0) + mass_ * g_;
  moment_ << u_vec(1), u_vec(2), u_vec(3);
*/
  //****************************** END ********************************************//

  Eigen::AngleAxisd yawAngle(des_euler(2), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rollAngle(des_euler(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(des_euler(1), Eigen::Vector3d::UnitY());
  orientation_ = yawAngle * rollAngle * pitchAngle;
}

const Eigen::Vector3d LinearControl::getComputedForce(void)
{
  return force_;
}

const Eigen::Vector3d LinearControl::getComputedMoment(void)
{
  return moment_;
}

const Eigen::Quaterniond &LinearControl::getComputedOrientation(void)
{
  return orientation_;
}
