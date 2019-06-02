#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

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

  /**
   * Init Initializes Kalman filter
   * @param mu_in Initial expected state
   * @param Q_in Inital state covariance/uncertanity matrix
   * @param P_in Initial process covariance
   * @param F_in State transition matrix
   * @param H_in state to measurement matrix
   * @param M_in Measurement covariance/uncertainty matrix
   */
  void Init(Eigen::VectorXd &mu_in,Eigen::MatrixXd &Q_in,
            Eigen::MatrixXd &P_in,Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in,Eigen::MatrixXd &M_in);

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
  void Update(const Eigen::VectorXd &z);

  void UpdateEKF(const Eigen::VectorXd &z);
  
   // state mean vector
  Eigen::VectorXd mu_;

  // process covariance/uncert matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // state covariance/uncert matrix
  Eigen::MatrixXd Q_;

  // measurement projection matrix
  Eigen::MatrixXd H_;

  // measurement covariance/uncert matrix
  Eigen::MatrixXd M_;
};

#endif // KALMAN_FILTER_H_
