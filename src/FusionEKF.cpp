#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "math.h"

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
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    noise_ax_ = 9.0;
    noise_ay_ = 9.0;

    //state covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    //state transition matrix
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

    //process covariance matrix
    ekf_.Q_ = MatrixXd(4, 4);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() = default;

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
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      //range
      double rho = measurement_pack.raw_measurements_[0];

      // bearing
      double phi = measurement_pack.raw_measurements_[1];

      // rate
      double rho_dot = measurement_pack.raw_measurements_[2];

      //normalize phi for between -PI and PI
      while (phi > M_PI) {
        phi -= 2.0 * M_PI;
      }
      while (phi < -M_PI) {
        phi += 2.0 * M_PI;
      }

      double px = rho * cos(phi);

      double py = rho * sin(phi);

      double vx = rho_dot * cos(phi);

      double vy = rho_dot * sin(phi);

      cout << " rho = " << rho << endl;
      cout << " phi = " << phi << endl;
      cout << " rho_dot = " << rho_dot << endl;
      cout << " px = " << px << endl;
      cout << " py = " << py << endl;
      cout << " vx = " << vx << endl;
      cout << " vy = " << vy << endl;

      ekf_.x_ << px, py, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state
      // no velocity info for lidar measurement
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_radar_, R_laser_, ekf_.Q_);
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "EKF Initial Results" << ekf_.x_ << endl;
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
  cout << "Predicting" << endl;
  double delta_time = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // convert to seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  //update state transition matrix F
  ekf_.F_(0, 2) = delta_time;
  ekf_.F_(1, 3) = delta_time;

  double delta_time2 = pow(delta_time, 2);
  double delta_time3 = pow(delta_time, 3);
  double delta_time4 = pow(delta_time, 4);

  //update process covariance matrix Q
  ekf_.Q_ << (0.25 * delta_time4 * noise_ax_), 0, (0.5 * delta_time3 * noise_ax_), 0,
            0, (0.25 * delta_time4 * noise_ay_), 0, (0.5 * delta_time3 * noise_ay_),
          (0.5 * delta_time3 * noise_ax_), 0, (delta_time2 * noise_ax_), 0,
          0, (0.5 * delta_time3 * noise_ay_), 0, (delta_time2 * noise_ay_);

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  cout << "updating" << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
