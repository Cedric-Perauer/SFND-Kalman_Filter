#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
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
  void LidarPredict();

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void RadarPredict();

  void SigmaPointPrediction(Eigen::MatrixXd &Xsig_aug,double delta_t);
  void AugmentedSigmaPoints(Eigen::MatrixXd &Xsig_aug);
  void PredictMeanAndCovariance();
  Eigen::MatrixXd toPolar(const Eigen::VectorXd& x);
	Eigen::MatrixXd toCartesian(const Eigen::VectorXd& x);
  void UpdateState(const Eigen::VectorXd &x);
  void normalize(double &num);

  //introduce a timestamp
  double timestamp_; 
  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_aug_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // augmented state covariance matrix
  Eigen::MatrixXd P_aug_; 

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // augmented Sigma Points Matrix
  Eigen::MatrixXd Xsig_aug_; 

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  //NIS for radar
  double radar_NIS_; 

  //NIS for lidar
  double lidar_NIS_; 

  // Weights of sigma points
  Eigen::VectorXd weights_;

  //radar measurement dimensions
  int n_r_; 

  //lidar measurement dimension
  int n_l_; 

  // State dimension
  int n_x_;

  //measurement dimension 
  int n_z_; 

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  //measurement covariance matrix S
  Eigen::MatrixXd S_; 

  //mean predicted measurement
  Eigen::VectorXd z_pred_; 

  // measuremnt vector
  Eigen::VectorXd z_; 

  //Matrix for Sigma Points in measurement space; 
  Eigen::MatrixXd Z_sig_; 

  //measurement noise Lidar
  Eigen::MatrixXd R_lidar_; 
  //measurement noise Radar
  Eigen::MatrixXd R_radar_; 

  // measurement Matrix 
  Eigen::MatrixXd H_; 
};

#endif  // UKF_H