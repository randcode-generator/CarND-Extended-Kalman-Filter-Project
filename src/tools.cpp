#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);

  rmse << 0,0,0,0;

  if(estimations.size() == 0) {
    std::cout <<"size is 0"<<endl;
    exit(1);
  }

  if(estimations.size() != ground_truth.size()) {
    std::cout <<"estimation and ground truth not equal"<<endl;
    exit(1);
  }

  for(int i=0; i < estimations.size(); ++i){
    VectorXd r = estimations[i] - ground_truth[i];
    r = r.array() * r.array();
    rmse += r;
  }

  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    exit(1);
  }

  //compute the Jacobian matrix
  Hj << px/c2, py/c2, 0, 0,
        (-1 * py)/c1, px/c1, 0, 0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;

  return Hj;
}
