#include <fstream>
#include <cmath>

#include "kalman_filter/KalmanFilter.h"

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

using k_filter::KalmanFilter;


k_filter::KalmanFilter::KalmanFilter(std::string file_name) {
    
  ReadPosi(file_name);
  state = Eigen::VectorXd(6);
  state << 0, 0, 0, 0, 0, 0;
  if_init = false;
  p = Eigen::MatrixXd::Identity(p.rows(), p.cols());

}

void k_filter::KalmanFilter::ReadPosi(std::string file_name) {
    
  std::fstream fins(file_name, std::fstream::in);
  if (fins.is_open()) {
    std::string line;
    while (getline(fins, line)) {
      std::stringstream ss(line);
      double x, y, z;
      ss >> x >> y >> z;
      PosData3d uwb_in(Eigen::Vector3d(x, y, z), 0);
      uwb_input.push_back(uwb_in);
    }        
    std::cout << "UWB positions got" << std::endl;
  } else {
    std::cerr << "I don't have UWB position" << std::endl;
  }
  fins.close();

}

void KalmanFilter::Trilateration(const Eigen::Vector4d& distance) {
    
  Eigen::Vector3d location;  
  int n, rows;
  int count = 0;

  if (uwb_input.size() < 3) {
    std::cout << "The quantity of stations is not enough" << std::endl;
  }

  uwb_input[0]._dis = distance(0);
  uwb_input[1]._dis = distance(1);
  uwb_input[2]._dis = distance(2);
  uwb_input[3]._dis = distance(3);

  // "n" refers to quantity of UWB stations
  // "rows" refers to the quantity of equations
  n = uwb_input.size();
  rows = n*(n-1) / 2;
  Eigen::MatrixXd m(rows, 3);
  Eigen::VectorXd result(rows);

  for (size_t i = 0; i < n; i++) {

    double x1, x2, y1, y2, z1, z2;
    double d1, d2;
    x1 = uwb_input[i]._pos(0);
    y1 = uwb_input[i]._pos(1);
    z1 = uwb_input[i]._pos(2);
    d1 = uwb_input[i]._dis;

    for (size_t j = i+1; j < n; j++) {

      x2 = uwb_input[j]._pos(0);
      y2 = uwb_input[j]._pos(1);
      z2 = uwb_input[j]._pos(2);
      d2 = uwb_input[j]._dis;

      m(count, 0) = x1 - x2;
      m(count, 1) = y1 - y2;
      m(count, 2) = z1 - z2;
      result(count) = (pow(x1, 2) - pow(x2, 2) +
                       pow(y1, 2) - pow(y2, 2) +
                       pow(z1, 2) - pow(z2, 2) +
                       pow(d2, 2) - pow(d1, 2)) / 2;
	  count++;
    }    
  }

  location = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(result);
  state(0) = location(0);
  state(1) = location(1);
  state(2) = location(2);

  std::cout << "The state is [ "
            << "x : " << state(0) << "  "
            << "y : " << state(1) << "  " 
            << "z : " << state(2) << " ]" << std::endl;

}

Eigen::Matrix<double, 4, 6> KalmanFilter::GetHmatrix(Eigen::Vector3d& p) {
  Eigen::Matrix<double, 4, 6> h = Eigen::MatrixXd::Zero(4, 6);
  std::vector<double> denomi;
  for (size_t i = 0; i < uwb_input.size(); i++) {
    double d;
    d = sqrt(pow((uwb_input[i]._pos(0) - p(0)), 2) + 
             pow((uwb_input[i]._pos(1) - p(1)), 2) +
             pow((uwb_input[i]._pos(2) - p(2)), 2));
    denomi.push_back(d);
  }
  
  for (size_t i = 0; i < uwb_input.size(); i++) {
    h(i, 0) = (2 * (p(0) - uwb_input[i]._pos(0))) / (2 * denomi[i]);
    h(i, 1) = (2 * (p(1) - uwb_input[i]._pos(1))) / (2 * denomi[i]);
    h(i, 2) = (2 * (p(2) - uwb_input[i]._pos(2))) / (2 * denomi[i]);
  }
  return h;

}


Eigen::Vector3d KalmanFilter::DataFusion(const Eigen::Vector4d& distance) {

  if (!if_init) {
    if_init = true;
    Trilateration(distance);
    Eigen::Vector3d tmp_state = state.head(3);
    return tmp_state;    
  } else {
    Eigen::Matrix<double, 6, 6> q_t;
    q_t = 0.00001 * Eigen::MatrixXd::Identity(6, 6);
    Eigen::Matrix<double, 6, 6> f_t = Eigen::MatrixXd::Identity(6, 6);
    // f_t(1,4) = 
    // f_t(2,5) =
    // f_t(3,6) =
    state = f_t * state;// + wt;
    p = f_t * p * f_t.transpose() + q_t;
    Eigen::Vector3d tmp_v = state.head(3);
    Eigen::Matrix<double, 4, 6> h = GetHmatrix(tmp_v);
    Eigen::Matrix<double, 4, 4> r_t;
    r_t = 0.1 * Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd k_t;
    k_t = p * h.transpose() * (h * p * h.transpose() + r_t).inverse();
    state = state + k_t * (distance - h * state);
    p = p - k_t * h * p;
    Eigen::Vector3d tmp_state = state.head(3);
    return tmp_state; 
  }
}