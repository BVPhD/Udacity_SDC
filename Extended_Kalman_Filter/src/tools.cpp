#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
    VectorXd temp = VectorXd(4);
    
    VectorXd rmse = VectorXd(4);
    rmse << 0.0,0.0,0.0,0.0;
    
    // accumulate squared residuals
    for (int i=0; i < estimations.size(); i++) {
        temp = ground_truth[i] - estimations[i];
        temp = temp.array()*temp.array();
        rmse = rmse + temp;
    }
    
    // take average
    rmse = rmse/estimations.size();
    // take sqrt
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
    MatrixXd Hj = MatrixXd(3,4);
    
    Hj <<   0.,0.,0.,0.,
            0.,0.,0.,0.,
            0.,0.,0.,0.;
    
    float p2 = x_state[0]*x_state[0] + x_state[1]*x_state[1];
    
    // check for near zero rho values.
    // if its non zero just fill with the derivative values
    if(p2 > 0.0000001){
        Hj(0,0) = x_state[0]/sqrt(p2);
        Hj(0,1) = x_state[1]/sqrt(p2);
        Hj(1,0) = -x_state[1]/p2;
        Hj(1,1) = x_state[0]/p2;
        Hj(2,0) = x_state[1]*(x_state[2]*x_state[1]-x_state[3]*x_state[0])/pow(p2,1.5);
        Hj(2,1) = x_state[0]*(x_state[3]*x_state[0]-x_state[2]*x_state[1])/pow(p2,1.5);
        Hj(2,2) = x_state[0]/sqrt(p2);
        Hj(2,3) = x_state[1]/sqrt(p2);
    }
    
    return Hj;
}
