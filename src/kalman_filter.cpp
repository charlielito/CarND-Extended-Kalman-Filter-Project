#include "kalman_filter.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

void print_dims(const MatrixXd& mat);
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

   VectorXd h_x = VectorXd(3);
   double ro = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
   double ro_dot = (x_[0]*x_[2] + x_[1]*x_[3])/ro;
   double theta = atan2(x_[1],x_[0]); //atan2 makes sure theta is -pi, pi
  h_x << ro, theta, ro_dot;

  MatrixXd H_j = Tools::CalculateJacobian(x_); // is 3x4
  VectorXd y = z - h_x; // is 3x1

  const float PI = 3.14159256358979f;
  // Check theta still in range -pi , pi
  if (y[1] > PI)
    y[1] -= (2*PI);
  else if (y[1] < -PI)
    y[1] += (2*PI);

  // std::cout << "Hola1" << std::endl;
  MatrixXd Ht = H_j.transpose(); // is 4x3
  // P is 4x4 , R is 3x3
  MatrixXd S = H_j * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  // print_dims(P_);
  // print_dims(Ht);
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_j) * P_;

}

void print_dims(const MatrixXd& mat){
  std::cout << "Cols: " << mat.cols();
  std::cout << " Rows: " << mat.rows() << std::endl;
}