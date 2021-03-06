#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

Tools KalmanFilter::tools_ = Tools();

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_radar_in, MatrixXd &R_lidar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_radar_ = R_radar_in;
  R_lidar_ = R_lidar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
   MatrixXd Ft = F_.transpose();
   P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  UpdateKalmanOptimized(z, H_, R_lidar_, false);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  MatrixXd Hj = tools_.CalculateJacobian(x_);
  std::cout << "Hj calculated" << Hj << std::endl;
  UpdateKalmanOptimized(z, Hj, R_radar_, true);
}

void KalmanFilter::UpdateKalmanOptimized(const VectorXd &z_in, const MatrixXd &H_in, const MatrixXd &R_in, bool flag_ekf) {
    VectorXd y;
    double rho, phi, rho_dot;
    VectorXd x_polar = VectorXd(3);

    if (!flag_ekf) {
        VectorXd z_pred = H_ * x_;
        y = z_in - z_pred;
    }
    else {

        rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
        phi = atan2(x_[1], x_[0]);

        // protection from division by zero
        if (rho < 0.00001) {
            rho = 0.00001;
        }

        rho_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / rho;

        x_polar << rho, phi, rho_dot;

        std::cout << "x polar = " << x_polar << std::endl;
        y = z_in - x_polar;

        // normalize the angle between -pi to pi
        while(y(1) > M_PI){
            y(1) -= 2 * M_PI;
        }

        while(y(1) < -M_PI){
            y(1) += 2 * M_PI;
        }
        std::cout << "y = " << y << std::endl;

    }

    MatrixXd Ht = H_in.transpose();
    std::cout << "Ht = " << Ht << std::endl;

    MatrixXd S = H_in * P_ * Ht + R_in;
    std::cout << "S = " << S << std::endl;

    MatrixXd Si = S.inverse();
    std::cout << "Si = " << Si << std::endl;

    MatrixXd PHt = P_ * Ht;
    std::cout << "PHt = " << PHt << std::endl;

    MatrixXd K = PHt * Si;
    std::cout << "K = " << K << std::endl;


    //new estimate
    x_ = x_ + (K * y);
    std::cout << "x_ = " << x_ << std::endl;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    std::cout << "I = " << I << std::endl;

    std::cout << "H_in = " << H_in << std::endl;
    std::cout << "P_ = " << P_ << std::endl;


    P_ = (I - K * H_in) * P_;
    std::cout << "P_ = " << P_ << std::endl;

}
