#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define M_PI           3.14159265358979323846

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);
  x_ << 1, 1, 0, 0;

  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Q_ = MatrixXd(4, 4);
  Q_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  R_laser = MatrixXd(2, 2);
  R_laser << 0.0225, 0,
             0, 0.0225;

  R_radar = MatrixXd(3, 3);
  R_radar << 0.09, 0, 0,
             0, 0.0009, 0,
             0, 0, 0.09;

  H_laser = MatrixXd(2, 4);
  H_laser << 1, 0, 0, 0,
             0, 1, 0, 0;

  Hj_radar = MatrixXd(3, 4);
  Hj_radar << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0;

  long x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
}

KalmanFilter::~KalmanFilter() {}

bool isAngleInRange(float val) {
  return val > (-1)*M_PI && val < M_PI;
}

float normalize(float val) {
  return fmod(val, M_PI);
}

void KalmanFilter::SetSensorType(MeasurementPackage m) {
  if(m.sensor_type_ == MeasurementPackage::RADAR) {
    R_ = R_radar;
    H_ = Hj_radar;
  } else if(m.sensor_type_ == MeasurementPackage::LASER) {
    R_ = R_laser;
    H_ = H_laser;
  }
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(VectorXd y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateLaser(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  Update(y);
}

void KalmanFilter::UpdateRadar(const VectorXd &z) {
  H_ = Tools::CalculateJacobian(x_);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  VectorXd h_x = VectorXd(3);
  float angle = atan2(py,px);
  float c1 = sqrt(px*px + py*py);
  float c2 = (px*vx + py*vy)/c1;
  h_x << c1, angle, c2;
  VectorXd y = z - h_x;
  
  if(isAngleInRange(y[1]) == false) {
    y[1] = normalize(y[1]);
  }
  Update(y);
}
