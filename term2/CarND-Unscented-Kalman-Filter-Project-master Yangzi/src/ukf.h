#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  /**
   * Below is some instance variables missing in original header file
   */
  ///* predicted measurement mean
  VectorXd z_pred_mean_;
  
  ///* Predicted measurement covariance
  MatrixXd S_;
  
  ///* Cross correlation between state and measurement
  MatrixXd T_;
  
  ///* Kalman gain
  MatrixXd K_;
  
  ///* Sigma points num
  int n_sig_;
  
  ///* Augmented sigma points matrix
  MatrixXd Xsig_aug_;
  
  ///* Measurement sigma points matrix
  MatrixXd Zsig_;
  
  ///* Measurement covariance matrix for lidar
  MatrixXd R_;
  
  ///* Measurement matrix for lidar, we don't have this for radar for the model is nonlinear
  MatrixXd H_lidar_;
  
  ///* Used to calculate delta t
  long long previous_timestamp_;
  
  ///* Threshold for screening small abs values
  double eps_;
  
  ///* Threshold for small delta t to skip predict
  double short_time_sec_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
  
  /**
   * Below are some helper methods
   */
  void InitUsingFirstMeasurement(MeasurementPackage meas_package);
  
  void ConstructAugmentedSigmaPoints();
  
  void CTRVProcessModel(double delta_t, int i);
  
  void PredictMean();
  
  void PredictCov();
  
  void ConstructRadarMeasurementSigmaPoints();
  
  void ConstructLidarMeasurementSigmaPoints();
  
  void RadarMeasurementTransform(int i);
  
  void LidarMeasurementTransform(int i);
  
  void PredictMeasurementMean();
  
  void PredictMeasurementCov();
  
  void UpdateHelper(const VectorXd &y);
  
  void CalculateCorrelationT();
  
  void CalculateKalmanGainK();
  
  void UpdateMeasurement(VectorXd z);
  
  void initBasedOnMeasurement(VectorXd z);
};

#endif /* UKF_H */
