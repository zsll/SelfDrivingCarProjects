#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

/**
 * Unified code for both Kalman Filter and EKF
 */
void KalmanFilter::UpdateHelper(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  cout << "Lidar data: " << z << endl;
  VectorXd y = z - H_ * x_;
  UpdateHelper(y);
}

// Normalizing Angles: Adjust phi between -pi and pi
static double normalizeAngle(double phi) {
  return std::remainder(phi, 2 * M_PI);
}

/**
 * This is the h(x') function for equation y = z - h(x')
 * Convert 4d Cartesian coordinates [px, py, vx, vy] to 3d polar system coordinates [rho, phi, rho dot]
 */
static VectorXd fromCartesianToPolar(VectorXd predicted_state) {
  double px = predicted_state[0];
  double py = predicted_state[1];
  double vx = predicted_state[2];
  double vy = predicted_state[3];
  cout << "Cartesion coordinates: " << predicted_state << endl;
  VectorXd projected_state = VectorXd(3);
  
  double rho = sqrt(pow(px, 2) + pow(py, 2));
  if(rho < 1e-3) {
    // Can't get valid polar system coordinates, drop updating from this frame
    // This is crucial to make Radar Update to work, else original state will be like an outlier and make dataset 1 off halfway
    return projected_state;
  }
  double phi = atan2(py, px);
  double rho_dot = ((px * vx) + (py * vy))/rho;
  
  projected_state[0] = rho;
  projected_state[1] = phi;
  projected_state[2] = rho_dot;
  
  cout << "Polar system coordinates: " << projected_state << endl;
  return projected_state;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  cout << "Radar data: " << z << endl;
  VectorXd z_pred = fromCartesianToPolar(x_);
  if(z_pred.isZero(0)) {
    // Can't get valid polar system coordinates, drop updating from this frame
    return;
  }
  VectorXd y = z - z_pred;
  cout << "Before angle normalization: " << y << endl;
  y[1] = normalizeAngle(y[1]); // Normalizing Angles: Adjust phi between -pi and pi
  cout << "After angle normalization: " << y << endl;
  UpdateHelper(y);
}
