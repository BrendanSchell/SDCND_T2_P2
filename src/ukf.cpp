#include <iostream>
#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  is_initialized_ = false;
  // initial state vector
  x_ = VectorXd(5);
  x_ << 0, 0, 0, 0, 0;
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1000, 0, 0, 0, 0,
        0, 1000, 0, 0, 0,
        0, 0, 1000, 0, 0,
        0, 0, 0, 1000, 0,
        0, 0, 0, 0, 1000;
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  Hint: one or more values initialized above might be wildly off...
  */
  Xsig_pred_ = VectorXd(5);
  Xsig_pred_ << 0, 0, 0, 0, 0;
  time_us_ = 0.;
  weights_ = VectorXd(11);
  weights_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  NIS_radar_ = 0.;
  NIS_laser_ = 0.;
//set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;
}

UKF::~UKF() {}

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
  if (!is_initialized_){
    time_us_ = meas_package.timestamp_;
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      float x_cart = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      float y_cart = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
      if (x_cart == 0 or y_cart == 0){
        return;
      }
      x_ << x_cart, y_cart, 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      x_ << meas_package.raw_measurements[0], meas_package.raw_measurements[1],0 ,0, 0;
    }
    is_initialized = true;
    return;
  }

  //calculate time delta between current and previous timestamp
  float dt = (meas_package.timestamp_ - time_us_)/1000000.0;

  //Update state transition matrix


  //Update process noise covariance matrix

  //set previous timestamp to current timestamp
  time_us_ = meas_package.timestamp_;

  //perform prediction step
  this.Prediction(dt

  //Perform updates based on measurement type
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    this.UpdateRadar(meas_package);
  } 
  else{
    this.UpdateLidar(meas_package);
  }
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
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
}
