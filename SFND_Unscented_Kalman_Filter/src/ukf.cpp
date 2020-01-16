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
  

  //radar and lidar measurement dimensions
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
   x_aug_ = VectorXd::Zero(n_aug_);
   P_ = MatrixXd::Zero(n_x_,n_x_); 
   P_aug_ = MatrixXd::Zero(n_aug_,n_aug_);
   Xsig_pred_ = MatrixXd::Zero(n_x_,2*n_aug_+1);
   Xsig_aug_ = MatrixXd::Zero(n_aug_,2*n_aug_+1);


   //H matrix
   H_ = MatrixXd::Zero(2,5);
   H_(0,0) = 1.0;  
   H_(1,1) = 1.0; 

   //measurement noise covariance matrices
    R_radar_ = MatrixXd::Zero(3, 3);
    R_radar_ <<     std_radr_ * std_radr_, 0                         , 0,
                    0                    , std_radphi_ * std_radphi_ , 0,
                    0                    , 0                         , std_radrd_*std_radrd_;

    R_lidar_ = MatrixXd::Zero(2, 2);
    R_lidar_ <<     std_laspx_ * std_laspx_, 0 ,
                    0                      , std_laspy_ * std_laspy_ ;
   
    weights_ = VectorXd(2*n_aug_+1);
    weights_(0) = lambda_/(lambda_+n_aug_);
    for (int i=1; i<2*n_aug_+1; i++){
      weights_(i) = 1/(2*(n_aug_+lambda_));
    }



  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  double m_1 = meas_package.raw_measurements_[0];
  double m_2 = meas_package.raw_measurements_[1];

  if(is_initialized_== false)
  {  
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      x_ << UKF::toCartesian(meas_package.raw_measurements_); 
    }
    else
    {
      x_ << m_1,m_2,0,0,0; 
    }
    timestamp_ = meas_package.timestamp_;
    if(x_(0)>0 && x_(1)>0)
  {
    is_initialized_ = true; 
    return; 
  }
  }

  

  double delta_t = (meas_package.timestamp_ -  timestamp_); //delta t

  Prediction(delta_t);

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    if (sqrt(pow(m_1,2) + pow(m_2,2)) > 0.001){
      UpdateLidar(meas_package.raw_measurements_);  //Update Lidar
    }
  } else {
    if (fabs(m_1) > 0.001){
      UpdateRadar(meas_package.raw_measurements_); //Update Radar
    }
    }
  }
  


void UKF::Prediction(double delta_t) {
  SigmaPointPrediction(&Xsig_pred_,delta_t);
  AugmentedSigmaPoints(&Xsig_aug_);
  PredictMeanAndCovariance(&x_,&P_);
}

void UKF::UpdateLidar(const Eigen::VectorXd &z) {
  //measurement
  VectorXd y  = z - H_ * x_;
  MatrixXd S  = H_*P_* H_.transpose() +R_lidar_;
  MatrixXd K  = P_ *  H_.transpose() * S.inverse();

  //estimate
  x_ = x_ +(K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  //NIS
  lidar_NIS = (y).transpose()* S * (y);
}

void UKF::UpdateRadar(const Eigen::VectorXd &z) {
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z_);

  //create matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z_,n_z_);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  Eigen::VectorXd z_diff;

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
  

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    //fromCartesianToPolar(
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y); 
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate cross correlation matrix
  Tc.fill(0.0);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    S = S + weights_(i) * z_diff * z_diff.transpose();

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    // angle normalization
    while (z_diff(3)> M_PI) z_diff(3)-=2.*M_PI;
    while (z_diff(3)<-M_PI) z_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

   S = S + R_radar_;
  //print result
  //cout << "z_pred: " << std::endl << z_pred << std::endl;
  //cout << "z: " << std::endl << z << std::endl;

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd y = z - z_pred;

  //angle normalization
  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  //update state mean and covariance matrix
  x_ = x_ + K * y;
  P_ = P_ - K*S*K.transpose();

  //print result
  //cout << "Updated state x: " << std::endl << x << std::endl;
  //cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //Calculate NIS
  radar_NIS_ = y.transpose()* S * y;
}

void UKF::PredictMeanAndCovariance(Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out) {
  
  x_.fill(0.0);
  // predict state mean
  for(int i = 0; i < 2*n_aug_+1;i++)
  {
      x_ = x_+weights_(i)*Xsig_pred_.col(i);
  }
  
  // predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
 

  // write result
  *x_out = x_;
  *P_out = P_;
}


void UKF::SigmaPointPrediction(Eigen::MatrixXd* Xsig_out,double delta_t) {

  for (int i = 0; i< 2*n_aug_+1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // write result
  *Xsig_out = Xsig_pred_;
}


void UKF::AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out) {


  // create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  // create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  // create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L.col(i);
  }
  
  
  // print result
  std::cout << "Xsig_aug_ = " << std::endl << Xsig_aug_ << std::endl;

  // write result
  *Xsig_out = Xsig_aug_;
}


Eigen::MatrixXd UKF::toPolar(const Eigen::VectorXd& x){

  Eigen::VectorXd x_n(4);

  x_n << x(0)*cos(x(1)),  x(0)*sin(x(1)), x(2), 0;

  return x_n;
}

Eigen::MatrixXd UKF::toCartesian(const Eigen::VectorXd& x){

  Eigen::VectorXd x_n(4);
  double r = x(0);
  double angle = x(1);

  r =  sqrt(pow(x(0),2) + pow(x(1),2));
  angle =  atan(x(1)/x(0));

  if (x(3))
    {
		x_n(2) = ((x(0)*x(2))+(x(1)*x(3)))/r;
	}

  x_n << r, angle, x_n(2), 0;

  return x_n;
}
