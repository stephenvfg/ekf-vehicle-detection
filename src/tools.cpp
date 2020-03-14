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
  
  // Prevent any division by zero
  if (px == 0 && py == 0) {
      std::cout << "ERROR in Tools::CalculateJacobian -> Division by Zero" << std::endl;
      return Hj;
  }
  
  // Calculate helper variable
  float pxy = px*px+py*py;
  
  // Compute the Jacobian matrix
  Hj << px/sqrt(pxy), py/sqrt(pxy), 0, 0,
    (-1*py)/(pxy), px/(pxy), 0, 0,
    py*(vx*py-vy*px)/pow(pxy, 3/2), px*(vy*px-vx*py)/pow(pxy, 3/2), px/sqrt(pxy), py/sqrt(pxy);

  return Hj;
}
