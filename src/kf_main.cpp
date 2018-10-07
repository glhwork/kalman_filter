#include "ros/ros.h"

#include "kalman_filter/KalmanFilter.h"
#include "kalman_filter/KFRos.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kalman_filter");
  ros::NodeHandle n;

  std::string addr_1 = "/home/george/renov_ws/src/kalman_filter/data/uwb_position.txt";
  std::string addr_2 = "/home/george/renov_ws/src/kalman_filter/data/acceleration.txt";
  kf_ros::KFRos localize(addr_1, addr_2, n);

  ros::Subscriber range_sub = n.subscribe("uwb_dis_info", 100, 
                                          &kf_ros::KFRos::GetRangeCallback,
                                          &localize);
  ros::spin();
  
  return 0;
}