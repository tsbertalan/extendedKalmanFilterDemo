#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define SECONDSPERTIMESTAMPUNIT 1e-6

// Will need to:
// 1. initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
// 2. initialize the Kalman filter position vector with 
//    the first sensor measurements
// 3. modify the F and Q matrices prior to the prediction 
//    step based on the elapsed time between measurements
// 4. call the update step for either the lidar or radar 
//    sensor measurement. Because the update step for lidar 
//    and radar are slightly different, there are different 
//    functions for updating lidar and radar.

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  // Write out the H_laser measurment operator.
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  ekf_ = KalmanFilter();

  // State (x, y, vx, vy)
  VectorXd x = VectorXd::Zero(4);
  // State covariance
  MatrixXd P = MatrixXd::Identity(4, 4);
  // State transition operator
  MatrixXd F = MatrixXd(4, 4);
  // Measurement function (DEPENDS ON MODALITY)
  MatrixXd H = MatrixXd(4, 4);
  // State transition noise
  MatrixXd Q = MatrixXd::Zero(4, 4);

  ekf_.Init(
    x, P, F, H, R_laser_, Q
  );

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}



void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    previous_timestamp_ = SECONDSPERTIMESTAMPUNIT * measurement_pack.timestamp_;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double ro, theta, x, y;
      ro = measurement_pack.raw_measurements_[0];
      theta = measurement_pack.raw_measurements_[1];

      x = ro * cos(theta);
      y = ro * sin(theta);

      // "Although radar gives velocity data in the form of the range rate ,
      //  a radar measurement does not contain enough information 
      //  to determine the state variable velocities."
      // ro_dot = measurement_pack.raw_measurements_[2];
      // vx = ro_dot * cos(theta);
      // vy = ro_dot * sin(theta);
      ekf_.x_ << x, y, 0, 0; //vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_  << 
        measurement_pack.raw_measurements_[0], 
        measurement_pack.raw_measurements_[1],
        0,
        0;
    }


    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // Time is measured in seconds.
  double t = SECONDSPERTIMESTAMPUNIT * measurement_pack.timestamp_;
  double dt = t - previous_timestamp_;
  previous_timestamp_ = t;

  // Update the state transition matrix F according to the new elapsed time.
  ekf_.F_ << 
    1,  0,  dt, 0,
    0,  1,  0,  dt,
    0,  0,  1,  0,
    0,  0,  0,  1;

  // Update the process noise covariance matrix.
  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  // Large nax,nay means that the motion model is less trusted than the data;
  // small, more.
  double nax, nay, ax, bx, cx, ay, by, cy;
  nax = 3; 
  nay = 3;
  ax = pow(dt, 4) * pow(nax, 2) / 4;
  bx = pow(dt, 3) * pow(nax, 2) / 2;
  cx = pow(dt, 2) * pow(nax, 2);
  ay = pow(dt, 4) * pow(nay, 2) / 4;
  by = pow(dt, 3) * pow(nay, 2) / 2;
  cy = pow(dt, 2) * pow(nay, 2);

  ekf_.Q_ << 
    ax, 0,  bx, 0,
    0,  ay, 0,  by,
    bx, 0,  cx, 0,
    0,  by, 0,  cy;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  // print("x_");
  // print(ekf_.x_);
  // print("P_");
  // print(ekf_.P_);
}
