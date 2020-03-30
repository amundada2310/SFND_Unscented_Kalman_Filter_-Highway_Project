#include "ukf.h"
#include "Eigen/Dense"
#include<iostream>
#include<stdio.h>
#include<fstream>//added

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
    // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  // create example covariance matrix
  
  P_ <<    1,   0,    0,   0,   0,
           0,   1,    0,   0,   0,
           0,   0,    1,   0,   0,
           0,   0,    0,   0.5, 0,//0.25//0.5//0.22
           0,   0,    0,   0,   0.5;//0.25//0.5//0.22

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;//1.46

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.46;
  
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
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  
    // time when the state is true, in us
  time_us_ = 0;//initial time set 
  
    // set state dimension
  n_x = 5;
  
 // set augmented dimension
  n_aug = 7;
  
 // create matrix with predicted sigma points as columns
  Xsig_pred = MatrixXd(n_x, ((2*n_aug) + 1));
  
  // define spreading parameter
  lambda_ = 3 - n_aug;
  
 // set weights
  // set vector for weights
  weights = VectorXd(2*n_aug+1);
  weights.fill(0.5/(lambda_ + n_aug)); 
  weights(0) = lambda_/(lambda_ + n_aug);

  
  //the NIS meaurement value initialize for Radar
  NIS_radar = 0;
  
  //the NIS meaurement value initialize for LIdar
  NIS_laser = 0;
  
   // Initialize measurement noise covairance matrix for Radar
   R_radar = MatrixXd(3, 3);
   R_radar << std_radr_*std_radr_, 0, 0,
               0, std_radphi_*std_radphi_, 0,
               0, 0, std_radrd_*std_radrd_;

  // Initialize measurement noise covairance matrix for Lidar
   R_lidar = MatrixXd(2,2);
   R_lidar << std_laspx_*std_laspx_, 0,
               0, std_laspy_*std_laspy_;
  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  
  if (!is_initialized_) //if first measurement
  {
    std::cout << "Kalman Filter Initialization " << std::endl;

    // set the state with the initial location and zero velocity
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)//check if lidar measurement
    {// if yes then input the Px and Py value into the state vector
    x_ << meas_package.raw_measurements_[0], 
          meas_package.raw_measurements_[1], 
          0, //we do not have details regarding this value
          0,//we do not have details regarding this value
          0;//we do not have details regarding this value
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)//check if Radar Measurement
    {// if yes then input the Px and Py value into the state vector
    double rho = meas_package.raw_measurements_[0];//first radar measurement in rho- range
    double phi = meas_package.raw_measurements_[1];//second is the phi - bearing
    double rho_dot = meas_package.raw_measurements_[2]; 
      
      //double vx = rho_dot * cos(phi);
      //double vy = rho_dot * sin(phi);
      //double v  = sqrt(vx * vx + vy * vy);
      
        x_ << rho * cos(phi), //calculate and input the value  Px
              rho * sin(phi),//calculate and input the value Py
              0, //calculate and input the value v
              0,//we do not have details regarding this value
      		  0;//we do not have details regarding this value
    }
    
    else 
    {
     std::cout<<"NO MEASUREMENT RECORDED"<< std::endl;// if nothing recorded
    }

    time_us_= meas_package.timestamp_;//input the time value
    is_initialized_ = true;//turn this on as first measurement taken
    
    //cout << "x_initialize= " << x_ << endl;//print the initial state vector
    return;
  }
  
  //if not first measurement do as below
  
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;//calculate the time between the previous and current measurements
  time_us_ = meas_package.timestamp_;//input current time as the previous time 

  // prediction step
  Prediction(dt);//call the prediction step same from lidar and radar

  // measurement update step
  if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)// call measurement step check if from lidar
  {
  UpdateLidar(meas_package);//call the function and input the measurement obtained i.e. z vector
  }
  
   if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)// call measurement step check if from lidar
  {
  UpdateRadar(meas_package);//call the function and input the measurement obtained i.e. z vector
  }

}

void UKF::Prediction(double delta_t) 
{
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // 1. Create Augmented sigma points matrix starts
  
  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
 
  // create augmented mean state
  x_aug.head(5) = x_;//first 5 values same as the input values 
  x_aug(5) = 0;// augment 6 value '0'
  x_aug(6) = 0;//augment 7 value '0'

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;// first 5*5 same as the covariance input values
  P_aug(5,5) = std_a_*std_a_;//(6,6) value add noise acceleration variance
  P_aug(6,6) = std_yawdd_*std_yawdd_;//(7,7) add noise change yaw rate variance

  // create square root matrix P
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug; ++i) 
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda_+n_aug) * L.col(i);
  }
  
  
  // 1. Create Augmented sigma points matrix ends- this step created a augment Xsig (Xa,k|k)7*15 sigma point matrix used for further calculations
  
  // 2. Prediction step 
  	//2.1 Sigma points prediction starts

  for (int i = 0; i< 2*n_aug+1; ++i) 
  {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

   
    if (fabs(yawd) > 0.001)  // avoid division by zero //if travelling on curved path
    {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } 
    else // if travelling on starting line
    {
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
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
  // 2.1 sigma points prediction ends- this step creates a prediction step sigma points matrix Xsig_pred [Xk+1|k] 5*15 which is used to calculate mean and coviarnce in prediction step
  
  //2.2 Predict state mean and covariance matrix starts
   // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) 
  {  // iterate over sigma points
    x_ = x_ + weights(i) * Xsig_pred.col(i);//new predicted mean vector
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) 
  {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights(i) * x_diff * x_diff.transpose();//new predicted covariance matrix
  }
  
  //2.2 predict state mean and covariance ends- this step creates x_(Xk+1|k new predicted state mean vector of size 5*1) and P_(Pk+1|k new predicted state covariance matrix of size 5*5)
  
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  
    // 1 Predict lidar measurement mean and predit radar measurement covariance starts

  // set measurement dimension, lidar can measure Px and Py locations- only for lidar
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);

    // measurement model
    Zsig(0,i) = p_x;                       // px
    Zsig(1,i) = p_y;                      // py

  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; ++i) 
  {
    z_pred = z_pred + weights(i) * Zsig.col(i);// predicted measurement state matrix 2*1
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix

  S = S + R_lidar;//predicted measurement covariance matrix 2*2 
  
  //1. Predict radar measurement mean and predit radar measurement covariance ends- this step craetes measurement mean z_pred (zk+1|k 2*1 new measurement mean state vector) and S (Sk+1|k 2*2 new measurement state covariance matrix)
  
  //2. Update State vector and update covariance matrix starts

  // create vector for incoming lidar measurement
  VectorXd z = VectorXd(n_z);
  z <<
     meas_package.raw_measurements_[0],   // Px in m
     meas_package.raw_measurements_[1];   // Py in m

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for(int i=0; i<((2 * n_aug) + 1); i++)
  {
      VectorXd x1 = Xsig_pred.col(i)-x_;
      VectorXd z1 = Zsig.col(i)-z_pred;
      
      //angle normalization
    while (z1(1)> M_PI) z1(1)-=2.*M_PI;
    while (z1(1)<-M_PI) z1(1)+=2.*M_PI;

    while (x1(3)> M_PI) x1(3)-=2.*M_PI;
    while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
    
    Tc = Tc + (weights(i) * x1 * z1.transpose());
    
  }

  // calculate Kalman gain K;
  
  MatrixXd K = Tc * S.inverse();
  
  // update state mean and covariance matrix

    VectorXd z2 = z - z_pred;
    
    //angle normalization
    while (z2(1)> M_PI) z2(1)-=2.*M_PI;
    while (z2(1)<-M_PI) z2(1)+=2.*M_PI;
    
    x_ = x_ + (K*z2);// state mean update
    
    P_ = P_ - (K * S * (K.transpose()));// covariance update
  
    //2. Update State vector and update covariance matrix ends- this step updates of our believe of x_(xk+1|k+1 state mean vector measurement 2*1 vector) and  P_(Pk+1|k+1 state covarinace matrix measurement 2*2 vector) - wrt original measurment of the next location.
  
  //3. Calculate NIS normalized innovation squared to check consistency of the designed filter
  
 NIS_laser = z2.transpose() * S.inverse() * z2; 
  // (actual current measurement - calculated measurement) = z - z_pred
  //std::cout << NIS_laser << std::endl; 
  /*std::ofstream myfile;
  myfile.open("NIS_laser.txt");
  if (myfile.is_open())
  {
    myfile << NIS_laser;
    myfile << "\n";
  }*/
  
  std::ofstream log("NIS_laser.txt", std::ios_base::app | std::ios_base::out);//creating a new text file called NIS_radar

    log << NIS_laser;//add NIS calculated value 
    log << "\n";

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  
  // 1 Predict radar measurement mean and predit radar measurement covariance starts

  // set measurement dimension, radar can measure r, phi, and r_dot - only for radar
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; ++i) 
  {
    z_pred = z_pred + weights(i) * Zsig.col(i);// predicted measurement state matrix 3*1
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix

  S = S + R_radar;//predicted measurement covariance matrix 3*3 
  
  //1. Predict radar measurement mean and predit radar measurement covariance ends- this step craetes measurement mean z_pred (zk+1|k 3*1 new measurement mean state vector) and S (Sk+1|k 3*3 new measurement state covariance matrix)
  
  //2. Update State vector and update covariance matrix starts

  // create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
     meas_package.raw_measurements_[0],   // rho in m
     meas_package.raw_measurements_[1],   // phi in rad
     meas_package.raw_measurements_[2];   // rho_dot in m/s

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for(int i=0; i<((2 * n_aug) + 1); i++)
  {
      VectorXd x1 = Xsig_pred.col(i)-x_;
      VectorXd z1 = Zsig.col(i)-z_pred;
      
      //angle normalization
    while (z1(1)> M_PI) z1(1)-=2.*M_PI;
    while (z1(1)<-M_PI) z1(1)+=2.*M_PI;

    while (x1(3)> M_PI) x1(3)-=2.*M_PI;
    while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
    
    Tc = Tc + (weights(i) * x1 * z1.transpose());
    
  }

  // calculate Kalman gain K;
  
  MatrixXd K = Tc * S.inverse();
  
  // update state mean and covariance matrix

    VectorXd z2 = z - z_pred;
    
    //angle normalization
    while (z2(1)> M_PI) z2(1)-=2.*M_PI;
    while (z2(1)<-M_PI) z2(1)+=2.*M_PI;
    
    x_ = x_ + (K*z2);// state mean update
    
    P_ = P_ - (K * S * (K.transpose()));// covariance update
  
    //2. Update State vector and update covariance matrix ends- this step updates of our believe of x_(xk+1|k+1 state mean vector measurement 3*1 vector) and  P_(Pk+1|k+1 state covarinace matrix measurement 3*3 vector) - wrt original measurment of the next location.
  
    //3. Calculate NIS normalized innovation squared to check consistency of the designed filter
  
  NIS_radar = z2.transpose() * S.inverse() * z2; 
  //std::cout << NIS_radar << std::endl;
  // z2 (actual current measurement - calculated measurement) = z - z_pred
  /*std::fstream myfile;
  myfile.open("NIS_radar.txt", std::fstream::out);
    myfile << NIS_radar << std::endl;
    myfile.close();*/

    std::ofstream log("NIS_radar.txt", std::ios_base::app | std::ios_base::out);//creating a new text file called NIS_radar

    log << NIS_radar;//add the NIS calculated values
    log << "\n";
}