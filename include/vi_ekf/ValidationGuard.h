#ifndef __ValidationGuard_H__
#define __ValidationGuard_H__

#include "ros/ros.h"
#include <svo_msgs/Info.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>
#include <vi_ekf/KalmanFilter.h>


// requiest actions
#define NO_ACTION         0
#define SAFE_OUTPUT	  1
#define REQUEST_AUTO_INIT 2
#define FAILSAFE          3



// vision stages
#define PAUSED		0
#define FIRST_FRAME     1
#define SECOND_FRAME	2
#define TRACKING	3
#define RELOCALISING	4

class ValidationGuard
{
  public:
  ValidationGuard(KalmanFilter10* mainFilter, ros::Publisher* resetPublisher, ros::Publisher* indivPublisher);
  void reset();
  void setOffset(double px, double py, double pz);
  int commitOutput();
  
  ros::Subscriber sub_svo_info_;
  void svo_info_Cb(const svo_msgs::Info::ConstPtr& msgin);
  void imu_Cb(double* q);
  
  private:
  const int vision_initialised_;
  const double auto_reinit_sec_, imu_lost_sec_;
  const double vision_lost_sec_;
  const double scale_diverged_, scale_converged_, scale_converged__min_, scale_lost_sec_;
  
  bool   ready_;
  int    vision_init_,   vision_stage_, filter_init_;
  int    vision_status_, imu_status_,   filter_status_;
  double vision_timer_, vision_lost_duration_,  imu_timer_, filter_timer_, filter_scale_timer_, master_timenow_; // timers logs the time of last valid update
  double  imu_q_[4], filter_P_offset_[3];
  ros::Publisher* reset_publisher_;
  ros::Publisher* vector_pub_;
  KalmanFilter10* filter_pointer_;
  
  void checkIMU();
  void checkVision();
  void checkFilter();
  void autoReinit();
  
};

#endif