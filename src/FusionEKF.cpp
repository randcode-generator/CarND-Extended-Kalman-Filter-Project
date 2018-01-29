#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  noise_ax = 9;
  noise_ay = 9;
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
    cout << "EKF: " << endl;
    VectorXd *x = &ekf_.getx();
    (*x) << 0.5, 0.5, 0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      (*x) << ro * cos(theta), ro * sin(theta), 0, 0;
    } else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      (*x) << px, py, 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  ekf_.SetSensorType(measurement_pack);
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds

  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  MatrixXd *F = &ekf_.getF();
  (*F)(0,2) = dt;
  (*F)(1,3) = dt;

  MatrixXd *Q = &ekf_.getQ();
  (*Q)(0,0) = dt_4/4*noise_ax;
  (*Q)(0,2) = dt_3/2*noise_ax;
  (*Q)(1,1) = dt_4/4*noise_ay;
  (*Q)(1,3) = dt_3/2*noise_ay;
  (*Q)(2,0) = dt_3/2*noise_ax;
  (*Q)(2,2) = dt_2*noise_ax;
  (*Q)(3,1) = dt_3/2*noise_ay;
  (*Q)(3,3) = dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateRadar(measurement_pack.raw_measurements_);
  } else {
    ekf_.UpdateLaser(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.getx() << endl;
  cout << "P_ = " << ekf_.getP() << endl;
}
