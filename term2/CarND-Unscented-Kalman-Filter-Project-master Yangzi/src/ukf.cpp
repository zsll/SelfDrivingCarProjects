#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2; // Since this UKF filter lecture uses CTRV model, should have a relative small velocity acceleration

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;  // Since this UKF filter lecture uses CTRV model, should have a relative small yaw acceleration

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off... Yeah, that's the process ones
  */
  
  // State dimension
  n_x_= x_.rows();
  
  // Augmented state dimension
  n_aug_= n_x_ + 2;
  
  // Sigma points number
  n_sig_ = 2 * n_aug_ + 1;
  
  // 15 sigma points for 5 dimensional vecotrs
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);
  
  // 15 sigma points for 7 dimensional vecotrs
  Xsig_aug_ = MatrixXd(n_aug_, n_sig_);
  
  // Sigma point spreading parameter
  lambda_= 3 - n_aug_;
  
  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  
  weights_(0) = lambda_ / float(lambda_ + n_aug_);
  double common_weight = 1 / float(2 *(lambda_ + n_aug_));
  
  for(int i=1; i<weights_.size(); ++i) {
    weights_(i) = common_weight;
  }
  
  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);  //Just use symetric matrix with zero covariance between independent measurements
  
  // p_x*p_x + p_y*p_y < eps, it will cause issue for calcualting phi dot for radar measurement
  eps_ = 0.001;
  
  // Threshold for small delta t to skip predict
  // even if car speed is around 20/s (72 km/h), it will only move < 0.5m in 0.02s
  short_time_sec_ = 0.02;
  
  // lidar update follow the old linear approach
  H_lidar_ = MatrixXd(2, 4);
  H_lidar_ << 1, 0 ,0 ,0,
  0, 1 , 0, 0;
}

UKF::~UKF() {}

// Normalizing Angles: Adjust phi between -pi and pi
static double normalizeAngle(double phi) {
  return std::remainder(phi, 2 * M_PI);
}

void UKF::InitUsingFirstMeasurement(MeasurementPackage meas_package) {
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    /**
     Convert radar from polar to cartesian coordinates and initialize state.
     */
    float rho = meas_package.raw_measurements_[0];
    float phi = meas_package.raw_measurements_[1];
    // Convert polar system coordinates back to Cartesian coordinates
    x_ << rho*cos(phi), rho*sin(phi), 0, 0, 0;
    
    // p_x*p_x + p_y*p_y < eps, it will cause issue for calcualting phi dot for radar measurement
    // Thus skip initialization using current data
    if(pow(x_[0], 2) + pow(x_[1], 2) < eps_) {
      return;
    }
    
    // Radar got lower resolution so covariance is more
    P_ << 25, 0, 0, 0, 0,
    0, 25, 0, 0, 0,
    0, 0, 100, 0, 0,
    0, 0, 0, 100, 0,
    0, 0, 0, 0, 100;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    /**
     Initialize state.
     */
    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    // Lidar got better resolution so covariance is less
    P_ << 5, 0, 0, 0, 0,
    0, 5, 0, 0, 0,
    0, 0, 100, 0, 0,
    0, 0, 0, 100, 0,
    0, 0, 0, 0, 100;
  }
  previous_timestamp_ = meas_package.timestamp_;
  // done initializing, no need to predict or update
  is_initialized_ = true;
  return;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    /**
     TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // meas_package is first measurement
    InitUsingFirstMeasurement(meas_package);
  }
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = meas_package.timestamp_;
  
  Prediction(dt);
  
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else {
    UpdateLidar(meas_package);
  }
}

// Genrate Sigma Points
void UKF::ConstructAugmentedSigmaPoints() {
  VectorXd x_aug = VectorXd(7);
  
  // construct augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  
  // construct augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  // construct augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  
  cout << "Augmented state vector: " <<  x_aug << endl;
  cout << "Augmented Cov: " <<  P_aug << endl;
  
  // construct square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  cout << "L: " <<  L << endl;
  // construct augmented sigma points
  Xsig_aug_ = MatrixXd(n_aug_, n_sig_);
  Xsig_aug_.col(0)  = x_aug;  // First is mean state
  for (int i = 0; i< n_aug_; i++)
  {
    VectorXd disturb_mean = sqrt(lambda_ + n_aug_) * L.col(i);  // Add noise to generate sigma points
    Xsig_aug_.col(i + 1) = x_aug + disturb_mean;
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - disturb_mean;
  }
}

// Predict Sigma Points. Generate Xsig_pred_ i out of Xsig_aug_ i following the CTRV model
void UKF::CTRVProcessModel(double delta_t, int i) {
  if (delta_t > short_time_sec_) {
    //predicted state values
    double px = Xsig_aug_(0,i);
    double py = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yaw_rate = Xsig_aug_(4,i);
    double nu_velo_acc = Xsig_aug_(5,i);
    double nu_yaw_acc = Xsig_aug_(6,i);
    
    double x_acc_factor = 0.5 * nu_velo_acc * delta_t * delta_t * cos(yaw);
    double y_acc_factor = 0.5 * nu_velo_acc * delta_t * delta_t * sin(yaw);
    
    if (fabs(yaw_rate) > eps_) {
      Xsig_pred_(0,i) = px + v / yaw_rate * ( sin (yaw + yaw_rate * delta_t) - sin(yaw)) + x_acc_factor;
      Xsig_pred_(1,i) = py + v / yaw_rate * ( cos(yaw) - cos(yaw + yaw_rate * delta_t)) + y_acc_factor;
    }
    else {  // yaw_rate can't be denorminator if it's too small
      Xsig_pred_(0,i) = px + v * delta_t * cos(yaw) + x_acc_factor;
      Xsig_pred_(1,i) = py + v * delta_t * sin(yaw) + y_acc_factor;
    }
    Xsig_pred_(2,i) = v + nu_velo_acc * delta_t;
    Xsig_pred_(3,i) = yaw + yaw_rate * delta_t + 0.5 * nu_yaw_acc * delta_t * delta_t;
    Xsig_pred_(4,i) = yaw_rate + nu_yaw_acc * delta_t;
  } else {
    // if dt is too short, will just skip predict step, and directly use sigma points for update
    Xsig_pred_(0,i) = Xsig_aug_(0,i);
    Xsig_pred_(1,i) = Xsig_aug_(1,i);
    Xsig_pred_(2,i) = Xsig_aug_(2,i);
    Xsig_pred_(3,i) = Xsig_aug_(3,i);
    Xsig_pred_(4,i) = Xsig_aug_(4,i);
  }
}

// Predict Mean using weighted Sum
void UKF::PredictMean() {
  x_.fill(0.0);
  for(int i = 0; i < Xsig_pred_.cols(); ++i) {
    x_ += (weights_[i] * Xsig_pred_.col(i));
  }
}

// Predict Cov using weighted Sum
void UKF::PredictCov() {
  P_.fill(0.0);
  for(int i=0;i < Xsig_pred_.cols(); ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff[3] = normalizeAngle(x_diff[3]); // Normalizing Angles: Adjust phi between -pi and pi
    P_ = P_ + weights_[i] * x_diff * x_diff.transpose() ;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  // construct augmented mean vector
  ConstructAugmentedSigmaPoints();
  
  //predict sigma points
  for (int i = 0; i< n_sig_; i++)
  {
    CTRVProcessModel(delta_t, i);
  }
  
  PredictMean();
  PredictCov();
  cout << "Predicted x_: " << x_ << endl;
  cout << "Predicted P_: " << P_ << endl;
}

void UKF::ConstructRadarMeasurementSigmaPoints() {
  Zsig_.fill(0.0);
  //transform sigma points into measurement space
  for(int i=0; i<Xsig_pred_.cols(); ++i) {
    RadarMeasurementTransform(i);
  }
}

// From predicted sigma points to corresponding measurement sigma points
void UKF::RadarMeasurementTransform(int i) {
  VectorXd x_sigma_pred = Xsig_pred_.col(i);
  Zsig_.col(i).fill(0.0); // initialize with zero vecotr
  double px = x_sigma_pred(0);
  double py = x_sigma_pred(1);
  double v = x_sigma_pred(2);
  double psi = x_sigma_pred(3);
  
  Zsig_.col(i)(0) = sqrt(pow(px, 2) + pow(py, 2));
  if(Zsig_.col(i)(0) < eps_) {
    return;
  }
  Zsig_.col(i)(1) = atan2(py, px);
  Zsig_.col(i)(2) = ((px * v * cos(psi) + py * v * sin(psi)) / Zsig_.col(i)(0));
  cout << "Radar data sigma: " << Zsig_ << endl;
}

void UKF::ConstructLidarMeasurementSigmaPoints() {
  Zsig_.fill(0.0);
  //transform sigma points into measurement space
  for(int i=0; i<Xsig_pred_.cols(); ++i) {
    LidarMeasurementTransform(i);
  }
  cout << "Lidar data sigma: " << Zsig_ << endl;
}

// From predicted sigma points to corresponding measurement sigma points
void UKF::LidarMeasurementTransform(int i) {
  VectorXd x_sigma_pred = Xsig_pred_.col(i);
  Zsig_.col(i).fill(0.0); // initialize with zero vecotr
  // We can only update position for lidar measurements
  Zsig_(0,i) = x_sigma_pred(0);
  Zsig_(1,i) = x_sigma_pred(1);
}

void UKF::PredictMeasurementMean() {
  z_pred_mean_.fill(0.0);
  for(int i = 0; i < Zsig_.cols(); ++i) {
    z_pred_mean_ += (weights_(i) * Zsig_.col(i));
  }
  cout << "z_pred_mean_: " << z_pred_mean_ << endl;
}

void UKF::PredictMeasurementCov() {
  S_.fill(0.0);
  for(int i = 0; i< Zsig_.cols(); ++i) {
    VectorXd diff = Zsig_.col(i) - z_pred_mean_;
    if(Zsig_.col(i).rows() == 3) {  // Only needed for radar
      diff(1) = normalizeAngle(diff(1)); // Normalizing Angles: Adjust phi between -pi and pi
    }
    S_ += (weights_(i) * (diff * diff.transpose()));
  }
  S_ += R_;
  cout << "PredictMeasurementCov S_: " << S_ << endl;
}

void UKF::CalculateCorrelationT() {
  T_.fill(0.0);
  for(int i=0 ; i < 2 * n_aug_ + 1; ++i) {
    VectorXd diff_x = (Xsig_pred_.col(i) - x_);
    diff_x(3) = normalizeAngle(diff_x(3));
    
    VectorXd diff_z = Zsig_.col(i) - z_pred_mean_;
    if(Zsig_.col(i).rows() == 3) { // Only needed for radar
      diff_z(1) = normalizeAngle(diff_z(1)); // Normalizing Angles: Adjust phi between -pi and pi
    }
    T_ += (weights_(i) * (diff_x * diff_z.transpose()));
  }
  cout << "CalculateCorrelationT T_: " << T_ << endl;
}

void UKF::CalculateKalmanGainK() {
  // Use these to update the state x_ and P_ using the Kalman Gain
  K_.fill(0.0);
  K_ = T_ * S_.inverse();
  cout << "CalculateKalmanGainK K_: " << K_ << endl;
}

void UKF::UpdateMeasurement(VectorXd z) {
  CalculateCorrelationT();
  CalculateKalmanGainK();
  
  VectorXd z_diff = z - z_pred_mean_;
  if(z_diff.rows() == 3) { // Only needed for radar
    z_diff(1) = normalizeAngle(z_diff(1)); // Normalizing Angles: Adjust phi between -pi and pi
  }
  
  //update state mean and covariance matrix
  x_ = x_ + K_ * (z_diff);
  P_ = P_ - K_ * S_ * K_.transpose();
  cout << "Updated x_: " << x_ << endl;
  cout << "Updated P_: " << P_ << endl;
  cout << "NIS for radar: " <<  z_diff.transpose() * S_.transpose() * z_diff << endl;
}

void UKF::initBasedOnMeasurement(VectorXd z) {
  // 3 dimensional predicted measurement for radar, 2 dimensional for lidar
  z_pred_mean_ = VectorXd(z.rows());
  
  // 15 sigma points for 3 dimensional measurements for radar, 2 dimensional for lidar
  Zsig_ = MatrixXd(z.rows(), n_sig_);
  
  // 3 by 3 predicted covariance matrix for radar, 2 by 2 dimensional for lidar
  S_ = MatrixXd(z.rows(), z.rows());
  
  // 5 by 3 predicted covariance matrix for radar, 5 by 2 dimensional for lidar
  T_ = MatrixXd(x_.rows(), z.rows());
  
  // 5 by 3 predicted covariance matrix for radar, 5 by 2 dimensional for lidar
  K_ = MatrixXd(x_.rows(), z.rows());
  
  if(z.rows() == 2) {
    // R matrices for update step using lidar
    R_ = MatrixXd(2,2);
    R_ << std_laspx_*std_laspx_, 0,
    0, std_laspy_*std_laspy_;
  } else {
    // R matrices for update step using radar
    R_ = MatrixXd(3,3);
    R_ << std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0, std_radrd_*std_radrd_;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   TODO:
   
   Complete this function! Use lidar data to update the belief about the object's
   position. Modify the state vector, x_, and covariance, P_.
   
   You'll also need to calculate the lidar NIS.
   */
  Eigen::VectorXd z = meas_package.raw_measurements_;
  cout << "Lidar data: " << z << endl;
  initBasedOnMeasurement(z);
  ConstructLidarMeasurementSigmaPoints();
  PredictMeasurementMean();
  PredictMeasurementCov();
  UpdateMeasurement(z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  Eigen::VectorXd z = meas_package.raw_measurements_;
  cout << "Radar data: " << z << endl;
  initBasedOnMeasurement(z);
  
  ConstructRadarMeasurementSigmaPoints();
  PredictMeasurementMean();
  PredictMeasurementCov();
  UpdateMeasurement(z);
}
