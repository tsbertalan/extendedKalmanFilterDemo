#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "tools.h"


// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  I_ = MatrixXd::Identity(4, 4);
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;// + u_;
  P_ = F_ * P_ * F_.transpose();
}

void KalmanFilter::Update(const VectorXd &z) {
  MatrixXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // ro = z[0];
  // theta = z[1];
  // ro_dot = z[2];
  // UpdateEKF is meant for the RADAR case. Here you have to transform 
  // the state from Cartesian into radial coordinates in order to compute
  // the vector z_pred.
  // In the method Update for the LIDAR case, we can just do z_pred = H * x_.
  /**
    * update the state by using Extended Kalman Filter equations
  */
  double px, py, vx, vy;
  px = x_[0];
  py = x_[1];
  vx = x_[2];
  vy = x_[3];

  VectorXd z_pred = VectorXd(3);
  double sssqr = sqrt(px * px + py * py);
  z_pred << 
    sssqr,
    atan(py / px),
    px * vx + py * vy / sssqr;

  MatrixXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
}
