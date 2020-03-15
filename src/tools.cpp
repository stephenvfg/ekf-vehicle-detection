#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  // Set up the RMSE vector
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Verify that the sizes of the estimation and ground truth vectors are appropriate
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "ERROR in Tools::CalculateRMSE -> Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }
  
  // Add the squared differences between ground_truth and estimations for every estimate
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the squared root of the mean and return the result
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  // Initialize the Jacobian matrix
  MatrixXd Hj(3,4);
  
  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Calculate helper variables
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  
  // Prevent any division by zero (or division by extremely small values)
  if (fabs(c1) < 0.0001) {
    std::cout << "ALERT in Tools::CalculateJacobian -> Division by Zero" << std::endl;
    c1 = 0.0001;
  }
  
  // Compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
