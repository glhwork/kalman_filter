#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

/*
   x_es : estimated state of the last time epoch
   x_pr : predicted state
   x    : final state after data fusion
   bt   : control input matrix
   ft   : state transmission matrix
   p_es : estimated convariance matrix
   p_pr : convariance matrix before data fusion (predicted)
   p    : convariance matrix after data fusion (determined)
   qt   : convariance matrix of process noise
   rt   : convariance matrix of measurement noise
   ht   : tansformation matrix between state and measurement
*/
void KalFilter(const Eigen::VectorXd x_es,
               const Eigen::VectorXd z) {
  Eigen::VectorXd x_pr;
  Eigen::VectorXd ut;
  Eigen::MatrixXd bt;
  Eigen::MatrixXd ft;
  Eigen::MatrixXd p_pr;
  Eigen::MatrixXd p_es;
  Eigen::MatrixXd qt;
  Eigen::MatrixXd rt;
  Eigen::MatrixXd ht;
  Eigen::MatrixXd kt;

  x_pr = ft * x_es + bt * ut;
  p_pr = ft * p_es * ft.transpose() + qt;
  kt = p_pr * ht.transpose() / (ht * p_pr * ht.transpose() + rt).inverse();
  
  Eigen::VectorXd x;
  Eigen::MatrixXd p;
  x = x_pr + kt * (z - ht * x_pr);
  p = p_pr - kt * ht * p_pr;
  
  
}


 


























int main() {
  Eigen::Matrix2d m1 = Eigen::Matrix2d::Random();
  Eigen::Matrix2d m2 = Eigen::Matrix2d::Random();
  Eigen::Matrix2d m3 = Eigen::Matrix2d::Random();

  Eigen::Matrix2d m4;
  m4 = m3 * (m2 + m1.transpose()).inverse();
  std::cout << m4 << std::endl;
  return 0;
}
