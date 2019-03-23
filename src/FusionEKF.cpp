#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  ekf_.R_radar_ = MatrixXd(3, 3);

  //measurement covariance matrix - laser
  ekf_.R_ = MatrixXd(2, 2);
  ekf_.R_ << 0.0225, 0,
            0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


  // measurement matrix lidar
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      // float ro_dot = measurement_pack.raw_measurements_[2];

      ekf_.x_[0] = ro * cos(theta);
      ekf_.x_[1] = ro * sin(theta);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }

    // Create the covariance matrix.
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.Q_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              1, 0, 1, 0,
              0, 1, 0, 1;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // 1. Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) =  dt;
  ekf_.F_(1,3) =  dt;
  // 2. Set the process covariance matrix Q
  float dt_2 = pow(dt,2);
  float dt_3 = pow(dt, 3);
  float dt_4 = pow(dt, 4);
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
            0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
            dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  // 4. Call the Kalman Filter update() function
  VectorXd z = VectorXd(2);
  VectorXd z_radar = VectorXd(3);


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    z_radar << measurement_pack.raw_measurements_[0], 
                measurement_pack.raw_measurements_[1],
                measurement_pack.raw_measurements_[2];
    ekf_.UpdateEKF(z_radar);

  } else {
    z << measurement_pack.raw_measurements_[0], 
                measurement_pack.raw_measurements_[1];
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
