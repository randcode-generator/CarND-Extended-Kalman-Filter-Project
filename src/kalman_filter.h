#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "measurement_package.h"

class KalmanFilter {
public:

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();
  
  Eigen::VectorXd &getx() {
    return x_;
  }

  Eigen::MatrixXd getP() {
    return P_;
  }

  Eigen::MatrixXd &getF() {
    return F_;
  }

  Eigen::MatrixXd &getQ() {
    return Q_;
  }  

  void SetSensorType(MeasurementPackage m);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateRadar(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateLaser(const Eigen::VectorXd &z);

private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  // measurement matrix (laser)
  Eigen::MatrixXd H_laser;

  // measurement covariance matrix (laser)
  Eigen::MatrixXd R_laser;

  // measurement matrix (radar)
  Eigen::MatrixXd Hj_radar;

  // measurement covariance matrix (radar)
  Eigen::MatrixXd R_radar;

  // Identity matrix of state vector(x_)
  Eigen::MatrixXd I;

  void Update(const Eigen::VectorXd y);
};

#endif /* KALMAN_FILTER_H_ */
