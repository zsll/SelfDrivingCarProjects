#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }
  
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    
    VectorXd residual = estimations[i] - ground_truth[i];
    
    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  
  //calculate the mean
  rmse = rmse/estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float c1 = px*px+py*py;
  
  // Divide by Zero throughout the Implementation, can't generate Jacobian Matrix, return all zero matrix
  /*
   Condsider the following equations for Measurement Update
   y = z − Hx′ (13)
   S=HP′HT+R (14)
   K = P′HT S−1 (15)
   x = x′ + Ky (16)
   P=(I−KH)P′ (17)
   When we can't get a valid Jacobian matrix, return all zero matrix as H
   We can easily derive x = x' and P = P'
   It's equivalent to drop this frame of observation
   To avoid unnecessary computation, we will check if the result is all zero matrix
   */
  if(fabs(c1) < 1e-3){
    cout << "Can't generate Jacobian Matrix due to dividion by zero." << std::endl;
    return Hj;
  }
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  
  Hj << (px/c2), (py/c2), 0, 0,
  -(py/c1), (px/c1), 0, 0,
  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  return Hj;
}
