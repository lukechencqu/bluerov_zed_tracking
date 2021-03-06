#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;


class KalmanFilter 
{

public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Predict Predicts the state and the state covariance
   *   using the process model
   */
  void Predict();

  /**
   * Updates the state and
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);
  

public:
  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // state transistion matrix
  MatrixXd F_;

  // process covariance matrix
  // Q越大，表面运动模型误差越大，滤波器越相信观测值
  // Q越小，越能抑制观测跳变点
  MatrixXd Q_;

  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  // R越大，表面传感器观测噪声越大，滤波器越相信预测值
  MatrixXd R_;

};

#endif  // KALMAN_FILTER_H_