#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Using KalmanFilter for linear functions - LIDAR
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // Calculate new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Convert p and v --> rho, phi, rhodot
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  
  // Prevent any division by zero (or division by extremely small values)
  double rho = sqrt(px*px + py*py);
  if (fabs(rho) < 0.001) {
    std::cout << "ALERT in KalmanFilter::UpdateEKF -> Division by Zero" << std::endl;
    rho = 0.001;
  }
  
  double phi = atan2(py, px);
  double rhodot = (px*vx + py*vy)/rho;
  
  // Create a new z_pred for non-linear functions
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rhodot;
  
  // Calculate y and adjust Phi to be between the accepted range of (-pi, pi)
  double pi_approx = 3.14159265358979323846;
  VectorXd y = z - z_pred;
  while (fabs(y[1]) > pi_approx) {
    std::cout << "ALERT in KalmanFilter::UpdateEKF -> Phi out of range" << std::endl;
    if (y[1] > pi_approx) { y[1] -= 2*pi_approx; }
    else { y[1] += 2*pi_approx; }
  }
  
  // Update the Kalman Filter business as usual
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // Calculate new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
