#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "tools.h"
#define PI 3.14159265359


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
  P_ = F_ * P_ * F_.transpose() + Q_;
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
    atan2(py, px),
    (px * vx + py * vy) / sssqr;

  if(fabs(sssqr) < 0.0001) {
    print("ERROR: Division by zero in radar measurment update.");
    print(z_pred);
  }

  VectorXd y = z - z_pred;

  // Wrap angle into -pi, pi range.
  // y[1] = fmod(y[1] + PI, 2*PI) - PI;
  if(y[1] > PI)
    y[1] -= 2*PI;
  if(y[1] < -PI)
    y[1] += 2*PI;
  if(y[1] < -PI || y[1] > PI) {
    print("ERROR: Radar measurment error angle phi out of range [-pi, pi]!");
    print("y[1] = ", "");
    print(y[1]);
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
}
