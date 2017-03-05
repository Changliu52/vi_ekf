#include <vi_ekf/ValidationGuard.h>

ValidationGuard::ValidationGuard(KalmanFilter10* mainFilter, ros::Publisher* resetPublisher, ros::Publisher* indivPublisher) :
  vision_initialised_(15),     // 15 continuous reading means a reliable initialiation
  auto_reinit_sec_(1.0),       // allow 2 second visual lost before conducting auto reinitialisation
  imu_lost_sec_(1.0),          // allow 1 second imu lost node
  vision_lost_sec_(1.0),       // allow 1 second vision lost node

  scale_diverged_(8.0),       // allow scale larger than this, to reset immediately
  scale_converged_(5.0),      // allow smaller than this value as reasonable
  scale_converged__min_(0.2), // allow larger  than this value as reasonable
  scale_lost_sec_(1.0),        // allow 3 second for filter scale to converge (not used)
  
  reset_publisher_(resetPublisher),
  vector_pub_(indivPublisher),
  filter_pointer_(mainFilter)  // saves the pointer to the kalman filter
{
  reset();
}

void
ValidationGuard::reset()
{
  ready_ = false; 
  vision_status_  = 0;   imu_status_ = 0;    filter_status_ = 0;
  vision_init_  = 0;     filter_init_ = 0;    vision_stage_ = 0;    
  double timenow = ros::Time::now().toSec();
  vision_timer_ = timenow;   imu_timer_   = timenow; filter_scale_timer_ = timenow;
  imu_q_[0] = 1.0; imu_q_[1] = 0.0; imu_q_[2] = 0.0; imu_q_[3] = 0.0;
  vision_lost_duration_ = 0.0;
}

void
ValidationGuard::setOffset(double px, double py, double pz)
{
  filter_P_offset_[0] = px;
  filter_P_offset_[1] = py;
  filter_P_offset_[2] = pz;
}


int
ValidationGuard::commitOutput()
{
  master_timenow_ = ros::Time::now().toSec();
  
  checkVision();
  checkIMU();
  checkFilter();
  
  // published vector for individual check for debug
  if (vector_pub_->getNumSubscribers()>0){
    geometry_msgs::Vector3 msgout;
    msgout.x = vision_status_;  msgout.y = imu_status_;  msgout.z = filter_status_;  vector_pub_->publish(msgout);
  }
  
  // Availability Check
  if ((vision_status_==REQUEST_AUTO_INIT || imu_status_==REQUEST_AUTO_INIT) || filter_status_==REQUEST_AUTO_INIT)
  {
    autoReinit();
    return REQUEST_AUTO_INIT;
  }
  if ((vision_status_==SAFE_OUTPUT && imu_status_==SAFE_OUTPUT) && filter_status_==SAFE_OUTPUT)
    return SAFE_OUTPUT;
  return NO_ACTION; 
}


void
ValidationGuard::svo_info_Cb(const svo_msgs::Info::ConstPtr& msgin)
{ 
  // message time interval
  double timeNow = ros::Time::now().toSec();
  double timeInterval = timeNow-vision_timer_;
  if (timeInterval < 0.0) timeInterval = 0.0;
  if (timeInterval > vision_lost_sec_) vision_status_ = NO_ACTION; // check dead SVO
  vision_timer_ = timeNow;
  
  // stages
  vision_stage_ = msgin->stage;
  switch(vision_stage_)
  {
    // In tracking stage
    case TRACKING:
      vision_lost_duration_  = 0.0; // reset relocalisation duration
      vision_status_         = SAFE_OUTPUT;
      // Initialisation check (ensure stable initialisation)
      if (vision_init_ < vision_initialised_) 
      {
        ++vision_init_;
	vision_status_ = NO_ACTION;
      }
      break;
    
    // In relocalisation stage
    case RELOCALISING:
      // check for automatic reinisitialisation
      vision_lost_duration_ += timeInterval;
      if (vision_lost_duration_ > auto_reinit_sec_) {
        vision_status_ = REQUEST_AUTO_INIT;
	std::cout<< "VISION guard requested automatic RESET" <<std::endl;
      } else {
        vision_status_ = NO_ACTION;
      }
      break;
    
    // Other stages (pause, first frame, second frame)
    default: 
      vision_lost_duration_  = 0.0;
      vision_status_ = NO_ACTION;
      vision_init_ = 0;
  }
}


void
ValidationGuard::imu_Cb(double* q)
{
  imu_q_[0] = q[0]; imu_q_[1] = q[1]; imu_q_[2] = q[2]; imu_q_[3] = q[3]; 
  imu_timer_ = ros::Time::now().toSec();
}




void 
ValidationGuard::checkIMU()
{
  if (master_timenow_-imu_timer_ > imu_lost_sec_){
    imu_status_ = REQUEST_AUTO_INIT;
    std::cout<< "IMU guard requested automatic RESET" <<std::endl;
  } else {
    imu_status_ = SAFE_OUTPUT;
  }
}


void
ValidationGuard::checkVision()
{
  if (master_timenow_-vision_timer_ > vision_lost_sec_)
    vision_status_ = FAILSAFE;
}


void
ValidationGuard::checkFilter()
{  
   double pTrace = filter_pointer_->Pmat.trace();
   double estError = filter_pointer_->estError_.sum();
   //std::cout<< "estError " << estError << " lambdaP " << filter_pointer_->Pmat(9,9) << " lambda " << filter_pointer_->state(9) <<std::endl;
//   if (pTrace < filter_diverged_) {  // allow level of uncertainty
//     filter_timer_ = master_timenow_;

   // Do not interfere with vision initialisation
   if  (vision_status_!=SAFE_OUTPUT) {  // before vision initialisation, no action
     filter_timer_ = master_timenow_;
     filter_status_ = NO_ACTION;
   } 
   // vision initialised and vision think it was fine, then check scale.
   else {  
     // 1. scale converged [0.3, 3.0]
     if (filter_pointer_->state(9) >=scale_converged__min_ && filter_pointer_->state(9) <=scale_converged_) { 
       filter_timer_ = master_timenow_;
       filter_status_ = SAFE_OUTPUT;
     }
     // 2. scale not yet converged (allow 'scale_lost_sec_' before reset) (3.0, 5.0)
     else if (filter_pointer_->state(9) < scale_diverged_ && filter_pointer_->state(9) >scale_converged_) { 
       // not for so long
       if (master_timenow_-filter_timer_ < scale_lost_sec_){
         filter_status_ = NO_ACTION;
       // If diverge for so long, request reset
       } else { 
         filter_timer_ = master_timenow_;
       	 filter_status_ = REQUEST_AUTO_INIT;
	 std::cout<< "FILTER Scale requested automatic RESET" <<std::endl;
       }
     }
     // 3. scale completely lost (reset immediately) (-x,0.3)||[5.0 +x)
     else {					
       filter_timer_ = master_timenow_;
       filter_status_ = REQUEST_AUTO_INIT;
       std::cout<< "FILTER Scale requested automatic RESET" <<std::endl;
     }
   }
/*   } else if (master_timenow_-filter_timer_ > filter_diverged_sec_){  //takes action when divered for too long
     filter_timer_ = master_timenow_;
     if  (vision_status_!=SAFE_OUTPUT) {  // before vision initialisation, no action
       filter_status_ = NO_ACTION;
     } else {  // vision initialised and vision think it was fine, then request auto-reset.
       filter_status_ = NO_ACTION;
       //std::cout<< "FILTER Covar requested automatic RESET" <<std::endl;
     }
   } else { // diverged but not so long, happens when vision just initialised, and before filter fully converging
     filter_status_ = NO_ACTION;
   }
*/
}


void
ValidationGuard::autoReinit()
{
  // get initial position
  geometry_msgs::TransformStamped msgout;
  msgout.transform.rotation.w = imu_q_[0]; msgout.transform.rotation.x = imu_q_[1]; msgout.transform.rotation.y = imu_q_[2]; msgout.transform.rotation.z = imu_q_[3];
  msgout.transform.translation.x = 0.0; msgout.transform.translation.y = 0.0; msgout.transform.translation.z = 0.0;
    
  // Publish reset message
  reset_publisher_->publish(msgout);
}
