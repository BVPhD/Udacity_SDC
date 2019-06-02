#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Update the state belief parameters for the sensor fusion EKF object
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Extended Kalman Filter object will be used for sensor fusion.
   */
  KalmanFilter ekf_;

 private:
  // check if the state believe parameters have been initialized
  bool is_initialized_;
  bool second_obs_;

  // previous timestamp large integer
  long long previous_timestamp_;

  // tool object used to compute measurment projection Jacobian and RMSE
  Tools tools;
  
  // measurement uncertainty matrices variable declarations
  Eigen::MatrixXd M_laser_;
  Eigen::MatrixXd M_radar_;
  
  //measurement state-to-observation projection matrices declerations
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd dH_radar_; // linearized observations to state space matrix
};

#endif // FusionEKF_H_
