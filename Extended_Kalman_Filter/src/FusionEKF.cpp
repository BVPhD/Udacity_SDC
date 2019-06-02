#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  second_obs_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  M_laser_ = MatrixXd(2, 2); //px,py
  M_radar_ = MatrixXd(3, 3); //rho,phi,rhodot
  H_laser_ = MatrixXd(2, 4); // (px,py,vx,y)->(px,py,0,0)

  // measurement covariance matrix - laser
  // this would come from measurement calibration in a lab
  M_laser_ << 0.0225, 0, // px measurement accuracy
              0, 0.0225; // py measurement accuracy

  // measurement covariance matrix - radar
  // this would come from measurement calibration in a lab
  M_radar_ << 0.09, 0, 0, // rho measurement accuracy
              0, 0.0009, 0, // phi measurement accuracy
              0, 0, 0.09; // rhodot measurement accuracy

  //project the state vector to the oberservation space of LIDAR measurments
  H_laser_ << 1,0,0,0,
              0,1,0,0;

  // dH_radar_ is state dependent and will need to be evaluated for each measurement processed
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  float vx = 0.0;
  float vy = 0.0;
  float px = 0.0;
  float py = 0.0;
    
  if (!is_initialized_) {

    // first measurement incorporation
    cout << "EKF initializing on first measurement..." << endl;
    ekf_.mu_ = VectorXd(4); // set the size of the state space vector
    ekf_.mu_ << 1., 1., 1., 1.; // initialize placeholder values
    ekf_.Q_ = MatrixXd(4,4);
    
    
    // set the dimensions of the evolution and process matrices
    ekf_.F_ = MatrixXd(4,4);
    ekf_.P_ = MatrixXd(4,4);

    // if the first measurement was from radar convert observations to
    // state space variables
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        cout << "RADAR measurement acquired." << endl;
        // initialize state, observations are {rho,phi,rhodot}
        float rho = measurement_pack.raw_measurements_[0];
        float phi =  measurement_pack.raw_measurements_[1];
        float rhodot =  measurement_pack.raw_measurements_[2];
 
        // Initialize state expected state:
        // If rhodot is not zero the true velocity is likely not zero,
        // you can use this information to incoorporate the most likely
        // value for px_dot or py_dot.

        // Im going to initialize
        // the velocity along the x direction.
        // I will change the direction
        // after the second observation.
        float pxdot = rhodot;
        float pydot = 0.0;

        ekf_.mu_ << rho*cos(phi),
                    rho*sin(phi),
                    pxdot,
                    pydot;
        
        
        float init_v_uncert = M_radar_(0,0);
        // technically we have no information
        // about the certainty of the state, but
        // it should be reasonably comparable
        // to the measurement uncertainty.

        ekf_.Q_ << M_radar_(0,0),0.,0.,0.,
                   0.,M_radar_(0,0),0.,0.,
                   0.,0.,init_v_uncert,0.,
                   0.,0.,0.,init_v_uncert;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        cout << "LIDAR measurement acquired." << endl;
        
        //Initialize state expectation
        ekf_.mu_ << measurement_pack.raw_measurements_[0],
                   measurement_pack.raw_measurements_[1],
                   0.,
                   0.;
        
        // the initial uncertainty will be large for the velocity but the position
        // uncertainty can be no larger than the measurement
        float init_v_uncert = M_laser_(0,0)*M_laser_(0,0)+M_laser_(1,1)*M_laser_(1,1);
  
        init_v_uncert = sqrt(init_v_uncert);
        ekf_.Q_ << M_laser_(0,0),0.,0.,0.,
                   0.,M_laser_(1,1),0.,0.,
                   0.,0.,init_v_uncert,0.,
                   0.,0.,0.,init_v_uncert;
    }

    // done initializing, no need to predict or update
    cout << "state vector and uncertainty initialized." << endl;
    cout << "mu_0 " << ekf_.mu_ << endl;

    second_obs_ = true;
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_; // store for next iteration
    
    return;
  }

  /**
   * Prediction
   */

  //in general this could be state/time dependent but is constant in this case
  float noise_ax = 9.; // process noise -- for x dimension, random acceleration noise uncertainty
  float noise_ay = 9.; // process noise -- for y dimension, random acceleration noise uncertainty
  float dt; //dt measured in seconds

  dt = (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
  
  // revise initial velocity measurements
  // given a second measurement

  // we only revise the initial velocity expectation
  // values from the second observation
  if (second_obs_){

   // the second measurement can be used, based on obs typ
   // to estimate the initial velocity before a prediction
   // is made
   if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        cout << "RADAR measurement 2 acquired." << endl;
        // initialize state, observations are {rho,phi,rhodot}
        float rho = measurement_pack.raw_measurements_[0];
        float phi =  measurement_pack.raw_measurements_[1];
        //float rhodot =  measurement_pack.raw_measurements_[2];

        // compute observation positions
        px = rho*cos(phi);
        py = rho*sin(phi);
    }else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        cout << "LIDAR measurement 2 acquired." << endl;
        //Initialize state with same assumptions for radar measurment, {px,py,0,0}
        px = measurement_pack.raw_measurements_[0];
        py = measurement_pack.raw_measurements_[1];
    }
   
   // estimate the expected velocity values and
   // update the expected state variables
   vx = (px - ekf_.mu_[0])/dt;
   vy = (py - ekf_.mu_[1])/dt;

   ekf_.mu_[2] = vx;
   ekf_.mu_[3] = vy;

   cout << "initial state vector changed based on second measurement." << endl;
   cout << "mu_0 " << ekf_.mu_ << endl;

   //now that a more accurate measure of the velocity has been set, 
   //continue with the standard Kalman filter algorithm
   second_obs_ = false;
  }

  
  // state evolution matrix, assuming constant velocity motion for dt seconds
  ekf_.F_ << 1,0,dt,0,
             0,1,0,dt,
             0,0,1,0,
             0,0,0,1;

  // process noise covariance matrix, under the assumption of
  // random acclerations in the x and y dimensions.
  ekf_.P_ << dt4*noise_ax/4., 0., dt3*noise_ax/2., 0.,
             0., dt4*noise_ay/4., 0., dt3*noise_ay/2.,
             dt3*noise_ax/2., 0., dt2*noise_ax, 0.,
             0., dt3*noise_ay/2., 0., dt2*noise_ay;

  // from the previous step state belief, evolve the system forward dt seconds.
  ekf_.Predict();
  
  /**
   * Measurement Incorporation with dependency on measurement type.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates need to change the dimensions
    // and update the measurement values
    ekf_.M_ = MatrixXd(3,3);
    ekf_.H_ = MatrixXd(3,4);
      
    ekf_.M_ = M_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.mu_); //EKF extension of Kalman results
    
    // the only difference between UpdateEKF and Update in this
    // implementation is that y = z - h(mu) is computed exactly rather than
    // the application of H on mu, and H_ has been replaced by the linearization
    // of the projection function about mu_
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates, need to change the dimensions
    // and update the measurement matrices
    ekf_.M_ = MatrixXd(2,2);
    ekf_.H_ = MatrixXd(2,4);
    
    ekf_.M_ = M_laser_;
    ekf_.H_ = H_laser_;
    
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the filter parameters
  cout << "mu_ = " << ekf_.mu_ << endl;
  cout << "Q_ = " << ekf_.Q_ << endl;
}
