#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  

  //radar and lidar dimensions
  n_r_ = 3; 
  n_l_ = 2; 

  //dimension of augemented x vector
   n_aug_ = 7;   

   //Sigma spreading parameter
   lambda_ = 3 - n_aug_; 

   //set NIS to 0 
   radar_NIS_ = 0; 
   lidar_NIS = 0; 

  //init vectors and matrices
   x_ = VectorXd::Zero(n_x_); 
   weights_ = VectorXd::Zero(2*n_aug_+1);
   P_ = MatrixXd::Zero(n_x_,n_x_); 
   Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);

   //measurement noise covariance matrices
    R_radar_ = MatrixXd::Zero(3, 3);
    R_radar_ <<     std_radr_ * std_radr_, 0                         , 0,
                    0                    , std_radphi_ * std_radphi_ , 0,
                    0                    , 0                         , std_radrd_*std_radrd_;

    R_lidar_ = MatrixXd::Zero(2, 2);
    R_lidar_ <<     std_laspx_ * std_laspx_, 0 ,
                    0                      , std_laspy_ * std_laspy_ ;
   


  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}