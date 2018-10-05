#ifndef KFROS_H
#define KFROS_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "kalman_filter/Uwbdis.h"


#include "KalmanFilter.h"


namespace kf_ros {

class KFRos {
 public:
  KFRos(std::string file_name_1, 
        std::string file_name_2, 
        ros::NodeHandle n);
  KFRos() {};
  ~KFRos() {};
  void GetRangeCallback(const kalman_filter::Uwbdis& uwb_dis);

 private:
  double t;
  bool start;
  k_filter::KalmanFilter filter;
  ros::Publisher position_pub;
  ros::Publisher pose_pub;
  
  
  
};  // class KFRos

}  // namespace kf_ros




#endif