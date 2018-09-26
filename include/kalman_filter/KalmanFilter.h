#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>



namespace k_filter {

template <class T>
struct StationData {
    StationData (const T& pos, double dis)
        : _pos(pos), _dis(dis) { }
    T _pos;
    double _dis; 
};  // struct StationData

typedef StationData<Eigen::Vector3d> PosData3d;
typedef std::vector<PosData3d> PosDataVec3d;



class KalmanFilter {
 public:
    KalmanFilter(std::string file_name);
    KalmanFilter() {}
    ~KalmanFilter() {}
    void Trilateration(const Eigen::Vector4d& distance);
    void ReadPosi(std::string file_name);
    Eigen::Matrix<double, 4, 6> GetHmatrix(Eigen::Vector3d& p);
    Eigen::Vector3d DataFusion(const Eigen::Vector4d& distance);

 private:
    PosDataVec3d uwb_input;
    Eigen::VectorXd state;
    Eigen::Matrix<double, 6, 6> ft;
    Eigen::Matrix<double, 6, 6> p;
    bool if_init;
};  // class KalmanFilter

}  // namespace k_filter

#endif