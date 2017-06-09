#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
#include <iostream>

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
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;

	//new state
	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
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

	  float c1 = px*px + py*py;
	  while(fabs(c1) < 0.0001){
	    px += 0.001;
	    py += 0.001;
	    c1 = px*px + py*py;
	  }

	  float ro = sqrt(px*px + py*py);
	  float phi = atan2(py, px);
	  float ro_dot = (px*vx + py*vy)/ro;
	  VectorXd z_pred_polar = VectorXd(3);
	  z_pred_polar << ro, phi, ro_dot;

	  MatrixXd Hj_ = H_;
	  MatrixXd Hjt = Hj_.transpose();
	  VectorXd y = z - z_pred_polar;

	  if (y[1] > M_PI) {
	    y[1] = y[1] - 2*M_PI;
	  }
	  else if (y[1] < -M_PI) {
	    y[1] = y[1] + 2 * M_PI;
	  }


	  MatrixXd S = Hj_ * P_ * Hjt + R_;
	  MatrixXd Si = S.inverse();
	  MatrixXd PHjt = P_ * Hjt;
	  MatrixXd K = PHjt * Si;

	  // new estimate
	  x_ = x_ + (K * y);
	  long x_size = x_.size();
	  MatrixXd I = MatrixXd::Identity(x_size, x_size);
	  P_ = (I - K * Hj_) * P_;
}
