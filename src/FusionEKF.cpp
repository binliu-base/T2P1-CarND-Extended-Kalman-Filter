#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //state covariance matrix P

  ekf_.P_ = MatrixXd(4, 4);

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);

  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    float px,py,vx,vy = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        float ro = measurement_pack.raw_measurements_(0);
        float phi = measurement_pack.raw_measurements_(1);
        float ro_dot = measurement_pack.raw_measurements_(2);

        px = ro * cos(phi);
        py = ro * sin(phi);
        vx = ro_dot * cos(phi);
        vy = ro_dot * sin(phi);

        //state covariance matrix P initialization
        ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        px = measurement_pack.raw_measurements_[0];
        py = measurement_pack.raw_measurements_[1];
        vx = 1;
        vy = 1;

        //state covariance matrix P initialization
        ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    }
	previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.x_ << px, py, vx, vy;

    if(px == 0 || py ==0){
      //will start true kalman state initialization till records whose px is not zero arrives
      return;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   //compute the elapsed time between the current and previous measurements
    float dt = (measurement_pack.timestamp_-previous_timestamp_) / 1000000.0; //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    //acceleration noise components
    float noise_ax = 5;
    float noise_ay = 5;

    //Modify the F matrix so that the time is integrated

    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //Set the process covariance matrix Q
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
      0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
      dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
      0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

	if (Tools::trace_tag == true) {
		int xxx = 1;
	}

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    R_radar_ << 0.03, 0, 0,
      0, 0.000001, 0,
      0, 0, 0.01;

    Tools tools;

    try {

      //when px is 0 or px*px + py*py is close to zero, we skip the update part for this radar measure

      //in order to avoid divide by zero situation

      Hj_ = tools.CalculateJacobian(ekf_.x_);

	  if (Tools::trace_tag == true) {
		  Tools::traceStream << "FusionEKF.cpp  void FusionEKF::ProcessMeasurement( " << endl;
		  Tools::traceStream << "Hj_=" << endl;
		  Tools::traceStream << Hj_ << endl;
		  Tools::traceStream << endl;

	  }
	  

    }
    catch (StringException & caught) {

      cout << "Got " << caught.what() << std::endl;
	    
      return;

    }
    

    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);    
  } else {

    // Laser updates
    R_laser_ << 0.0225, 0,
      0, 0.0225;
    H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;

  if (Tools::trace_tag == true) {
	  Tools::traceStream << "FunsionEKF.cpp -> void FusionEKF::ProcessMeasurement  " << endl;
	  Tools::traceStream << "ekf_.x_" << endl;
	  Tools::traceStream << ekf_.x_ << endl;
	  Tools::traceStream << endl;

	  Tools::trace_tag = false;
  }
}
