#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 6.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
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
  
  timestamp_ = 0; 
  //radar and lidar measurement dimensions
  n_r_ = 3; 
  n_l_ = 2; 
  n_x_ = 5; 
  n_aug_ = 7; 

   //Sigma spreading parameter
   lambda_ = 3 - n_aug_; 

   //set NIS to 0 
   radar_NIS_ = 0; 
   lidar_NIS_ = 0; 

  //init vectors and matrices
   x_ = VectorXd::Zero(n_x_); 
   P_ = MatrixXd::Zero(n_x_,n_x_); 
   Xsig_pred_ = MatrixXd::Zero(n_x_,2*n_aug_+1);
   Xsig_aug_ = MatrixXd::Zero(n_aug_,2*n_aug_+1);
   weights_ = VectorXd::Zero(2*n_aug_+1);
   P_aug_ = MatrixXd::Zero(n_aug_,n_aug_);
   x_aug_ = VectorXd::Zero(n_aug_);

   //measurement noise covariance matrices
    R_radar_ = MatrixXd::Zero(3, 3);
    R_radar_ <<     std_radr_ * std_radr_, 0                         , 0,
                    0                    , std_radphi_ * std_radphi_ , 0,
                    0                    , 0                         , std_radrd_*std_radrd_;

    R_lidar_ = MatrixXd::Zero(2, 2);
    R_lidar_ <<     std_laspx_ * std_laspx_, 0 ,
                    0                      , std_laspy_ * std_laspy_ ;
}

UKF::~UKF() {}


void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  float m1 = static_cast<float>(meas_package.raw_measurements_(0));
  float m2 = static_cast<float>(meas_package.raw_measurements_(1));
  
 if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == false)
    {
      return;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ ==false )
    {
      return;
    }

  if(is_initialized_== false)
  {  
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      x_ << m1*cos(m2),m1*sin(m2),0,0,0; 
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << m1,m2,0,0,0; 
    }
    
    timestamp_ = meas_package.timestamp_;
    is_initialized_ = true; 
    return;

  }

  double delta_t = (meas_package.timestamp_ -  timestamp_)*1e-6; //delta t
  timestamp_ = meas_package.timestamp_;


  Prediction(delta_t);

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
     n_z_ = n_r_; 

    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      n_z_ = n_l_; 
    }
  S_ = MatrixXd::Zero(n_z_,n_z_); //measuremnt covariance Matrix
  z_pred_ = VectorXd::Zero(n_z_); //mean prediction measure 
  Z_sig_ = MatrixXd::Zero(n_z_,2*n_aug_+1); //sigma points Matrix 
  VectorXd z_diff = VectorXd::Zero(n_z_);
   if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    { 
      RadarPredict();
      z_diff = meas_package.raw_measurements_-z_pred_;   
      radar_NIS_ = z_diff.transpose()*S_.inverse()*(z_diff);
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    { 
      LidarPredict();
      z_diff = meas_package.raw_measurements_-z_pred_; 
      lidar_NIS_ = z_diff.transpose()*S_.inverse()*(z_diff);
    }

  UpdateState(meas_package.raw_measurements_);
}
  


void UKF::Prediction(double delta_t) 
{ 
  AugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
}


void UKF::UpdateState(const VectorXd &z)
{ 
  
  MatrixXd Tc = MatrixXd::Zero(n_x_,n_z_); //init as Zero is important

  for(int i=0; i < 2*n_aug_+1;++i)
  {
    //residual
    VectorXd z_diff = Z_sig_.col(i) - z_pred_; 

    normalize(z_diff(1));

    VectorXd x_diff = Xsig_pred_.col(i) - x_; 

    normalize(x_diff(3));
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

  }

  //Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  //residual
  VectorXd z_diff = z - z_pred_;

  //angle normalization
  normalize(z_diff(1));
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_*K.transpose();
  
}




void UKF::LidarPredict() {
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    Z_sig_(0,i) = Xsig_pred_(0,i);
    Z_sig_(1,i) = Xsig_pred_(1,i);
    
  }

  //mean predicted measurement
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred_ = z_pred_ + weights_(i) * Z_sig_.col(i);
  }

  //calculate cross correlation matrix
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Z_sig_.col(i) - z_pred_;
    
    //angle normalization
    normalize(z_diff(1));
    S_= S_ + weights_(i) * z_diff * z_diff.transpose();
  }

   S_ = S_ + R_lidar_;
}


void UKF::RadarPredict() {
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
  

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    //fromCartesianToPolar
    Z_sig_(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Z_sig_(1,i) = atan2(p_y,p_x);                                 //phi
    Z_sig_(2,i) = (p_x*v1 + p_y*v2) / max(0.001,Z_sig_(0,i));     //r_dot
  }
  

  //mean predicted measurement
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred_ = z_pred_ + weights_(i) * Z_sig_.col(i);
  }

  //calculate cross correlation matrix
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Z_sig_.col(i) - z_pred_;
    //angle normalization
    normalize(z_diff(1));
    S_= S_ + weights_(i) * z_diff * z_diff.transpose();
  }

   S_ = S_ + R_radar_;

}


void UKF::AugmentedSigmaPoints() {
   
    Xsig_aug_.fill(0.0);
    //create augmented mean vector
    x_aug_.fill(0.0);
    //create augmented state covariance
    P_aug_.fill(0.0);

    //create augmented mean state
    x_aug_.head(n_x_) = x_;

    //create augmented covariance matrix
    P_aug_.topLeftCorner(n_x_, n_x_) = P_;
    P_aug_(n_x_, n_x_) = std_a_ * std_a_;
    P_aug_(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    // Cholesky decomposition
    MatrixXd L = P_aug_.llt().matrixL();

    //create augmented sigma points
    Xsig_aug_.col(0)  = x_aug_;
    for (int i = 0; i< n_aug_; i++) {
        Xsig_aug_.col(i+1)        = x_aug_ + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L.col(i);
    }
}


void UKF::SigmaPointPrediction(double dt) {
    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
        }
        else {
            px_p = p_x + v*dt*cos(yaw);
            py_p = p_y + v*dt*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*dt;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
        py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
        v_p = v_p + nu_a*dt;

        yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
        yawd_p = yawd_p + nu_yawdd*dt;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
}


void UKF::PredictMeanAndCovariance() {
    // set weights and these weights will be shared during update
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (lambda_ + n_aug_);
        weights_(i) = weight;
    }

    //predicted state mean
    x_.fill(0.0);
    //iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);
    //iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        normalize(x_diff(3));
        P_ += weights_(i) * x_diff * x_diff.transpose() ;
    }
}



void UKF::normalize(double &num)
{
  while(num>M_PI) num-=2.*M_PI; 
  while(num<-M_PI) num+=2.*M_PI; 

}
