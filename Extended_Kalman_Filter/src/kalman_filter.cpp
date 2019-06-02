#include "kalman_filter.h"
#include <math.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

// Initialization of the Kalman initial beliefs of the state
// And definitions of measurement and processes
void KalmanFilter::Init(VectorXd &mu_in,MatrixXd &Q_in,
                        MatrixXd &P_in,MatrixXd &F_in,
                        MatrixXd &H_in,MatrixXd &M_in){

  mu_ = mu_in; // initialize state expectation
  Q_ = Q_in; // initialize state uncertainty/covariance

  P_ = P_in; // set process noise covariance
  F_ = F_in; // set the state-to-state transition matrix
    
  H_ = H_in; // set the state-to-measurement projection matrix
  M_ = M_in; // set the measurement projection uncertainty/covariance
}

// Evolve the state expectation forward in time
// Evovle the state uncertainty convolved with the process uncertainty
void KalmanFilter::Predict() {
    MatrixXd Ft = F_.transpose();
    
    mu_ = F_*mu_; // update mean state prediction
    Q_ = P_ + F_*Q_*Ft; // update state uncertainty -> process noise + transformed previous uncertainty
}

// Integrate measurement into expectations and uncertainty of the state
void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = z - H_*mu_; // z-value for observation and expected observation value
    MatrixXd Ht = H_.transpose();
    
    // Auxliary matrix for making calculations
    MatrixXd S = M_ + H_*Q_*Ht;
    MatrixXd Si = S.inverse();
    
    MatrixXd K = Q_*Ht*Si; // Kalman Gain

    mu_ = mu_ + K*y; // update expected state from measurement
    Q_ = Q_ - K*H_*Q_; // update state uncertainty after measurement
}

void KalmanFilter::UpdateEKF(const VectorXd &z){
    float rho = sqrt(mu_[0]*mu_[0]+mu_[1]*mu_[1]);
    float pi = 3.141592654;
    
    VectorXd h = VectorXd(3);
    
    // check for divide by zero
    if(mu_[0]==0.0 && mu_[1]==0.0){
     //phi and rhodot shouldnt really be 0.0, but should be
     // approximated by the previous results.
     // previous state measure are approx ~
     // mu_prev = Fi*mu, where Fi is the same as F_ expect dt is replaced by -dt
     
     MatrixXd Fi = F_;
     Fi(0,2) = -F_(0,2);
     Fi(1,3) = -F_(1,3);
     VectorXd mu_prev = Fi*mu_; // compute the previous state
        
     rho = sqrt(mu_prev[0]*mu_prev[0]+mu_prev[1]*mu_prev[1]);
     //and if that is STILL zero just give zeros across the board
     if(rho == 0.0){
         h << 0.0,
              0.0,
              0.0;
     }
     else{
         // compute the previous expected values for phi and rhodot
         // and use those values as the expecation 
         h << 0.0, 
              atan2(mu_prev[1],mu_prev[0]), 
              (mu_prev[0]*mu_prev[2]+mu_prev[1]*mu_prev[3])/rho;
     }
    }
    else{
     // if rho isnt zero just use the non-linear functions
     // that map the state expectation to the observation variables
     h << rho, 
          atan2(mu_[1],mu_[0]), 
          (mu_[0]*mu_[2]+mu_[1]*mu_[3])/rho;
    }
    
    VectorXd y = z - h; // z-value for observation and expected observation value
    
    //check y[1] angle is properly formatted
    y[1] = fmod(y[1],2.*pi);
    
    if(y[1] > pi){
        y[1] = y[1]-2.*pi;
    }
    if(y[1] < -pi){
        y[1] = y[1]+2.*pi;
    }
    
    
    MatrixXd Ht = H_.transpose();
    
    // Auxliary matrix for making calculations
    MatrixXd S = M_ + H_*Q_*Ht;
    MatrixXd Si = S.inverse();
    
    MatrixXd K = Q_*Ht*Si; // Kalman Gain
    
    mu_ = mu_ + K*y; // update expected state from measurement
    Q_ = Q_ - K*H_*Q_; // update state uncertainty after measurement
}
