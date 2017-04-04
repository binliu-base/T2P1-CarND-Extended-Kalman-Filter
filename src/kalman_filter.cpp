#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <fstream>

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
	if (Tools::trace_tag == true) {
		Tools::traceStream << "KalmanFilter::Predict--> bPredict " << endl;
		Tools::traceStream << "x_=" << endl;
		Tools::traceStream << x_ << endl;
		Tools::traceStream << "P_=" << endl;
		Tools::traceStream << P_ << endl;
		Tools::traceStream << "Q_=" << endl;
		Tools::traceStream << Q_ << endl;
		Tools::traceStream << endl;
	}
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;  

  if (Tools::trace_tag == true) {
	  Tools::traceStream << "KalmanFilter::Predict--> ePredict " << endl;
	  Tools::traceStream << "x_=" << endl;
	  Tools::traceStream << x_ << endl;
	  Tools::traceStream << "F_=" << endl;
	  Tools::traceStream << F_ << endl;
	  Tools::traceStream << "Ft=" << endl;
	  Tools::traceStream << Ft << endl;
	  Tools::traceStream << endl;

  }
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  float px_b, py_b, vx_b, vy_b = 0;
  px_b = x_[0];
  py_b = x_[1];
  vx_b = x_[2];
  vy_b = x_[3];

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  if (Tools::trace_tag == true) {
	  Tools::traceStream << "kalman_filter.cpp ---> void KalmanFilter::Update " << endl;
	  Tools::traceStream << "z=" << endl;
	  Tools::traceStream << z << endl;
	  Tools::traceStream << "z_pred=" << endl;
	  Tools::traceStream << z_pred << endl;
	  Tools::traceStream << "y=" << endl;
	  Tools::traceStream << y << endl;
	  Tools::traceStream << "x_=" << endl;
	  Tools::traceStream << x_ << endl;
	  Tools::traceStream << "K=" << endl;
	  Tools::traceStream << K << endl;
	  Tools::traceStream << "P_=" << endl;
	  Tools::traceStream << P_ << endl;
	  Tools::traceStream << endl;
  }

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {  
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];


  float ro = sqrt(px * px + py *py);
  float phi = atan(py/px);
  float ro_dot = (px*vx + py *vy)/ro;

  VectorXd z_pred(3);
  z_pred << ro, phi, ro_dot;

  VectorXd y = z - z_pred;                
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;                //*****

  float px_b, py_b, vx_b, vy_b = 0;
  px_b = x_[0];
  py_b = x_[1];
  vx_b = x_[2];
  vy_b = x_[3];

  //new estimate
  x_ = x_ + (K * y);                    //*****
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;                

  if (Tools::trace_tag == true) {

    Tools::traceStream << "kalman_filter.cpp ---> void KalmanFilter::UpdateEKF " << endl;
	Tools::traceStream << "z=" << endl;
	Tools::traceStream << z << endl;
	Tools::traceStream << "z_pred=" << endl;
    Tools::traceStream << z_pred << endl;
	Tools::traceStream << "y=" << endl;
	Tools::traceStream << y << endl;
	Tools::traceStream << "x_=" << endl;
	Tools::traceStream << x_ << endl;
	Tools::traceStream << "K=" << endl;
	Tools::traceStream << K << endl;
	Tools::traceStream << "P_=" << endl;
	Tools::traceStream << P_ << endl;

  }  
}

