#include "ros/ros.h"
#include <vi_ekf/KalmanFilter.h>
#include <vi_ekf/HelperFunc.h>
#include <vi_ekf/ValidationGuard.h>
#include <vi_ekf/teensyPilot.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <vikit/output_helper.h>
#include <sophus/se3.h>


//____________________________
// IMU callback function
 void IMUCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin, 
		  tf::TransformBroadcaster& posPublisher, 
		  ros::Publisher& posIndicator, 
		  KalmanFilter10& mainFilter, 
		  double* q, 
		  ValidationGuard& guard,
		  ros::Publisher& publisher)
{
  // Compute rotation matrix
  q[0] = (double)msgin->transform.rotation.w; 
  q[1] = (double)msgin->transform.rotation.x; 
  q[2] = (double)msgin->transform.rotation.y; 
  q[3] = (double)msgin->transform.rotation.z;
  
  // update the imu dalay buffer
  Vector4d q_vec; q_vec << q[0], q[1], q[2], q[3];
  mainFilter.q_bf.push_back(q_vec);
  if (mainFilter.q_bf.size() == (mainFilter.delayed_imu_readings_+1)) {// num of delayed measurements + 1  
  //if (mainFilter.q_bf.size() == (4+1)) {// num of delayed measurements + 1  
    // delet old buffer elements
    mainFilter.q_bf.pop_front();
  } 
  
  // convert q to R
  double _2qxqx = 2*q[1]*q[1];  double _2qyqy = 2*q[2]*q[2];  double _2qzqz = 2*q[3]*q[3];
  double _2qxqy = 2*q[1]*q[2];  double _2qxqz = 2*q[1]*q[3];  double _2qxqw = 2*q[1]*q[0];
  double _2qyqz = 2*q[2]*q[3];  double _2qyqw = 2*q[2]*q[0];
  double _2qzqw = 2*q[3]*q[0];
  
  Matrix3d R;
  R<< (1 - _2qyqy - _2qzqz), (_2qxqy - _2qzqw),     (_2qxqz + _2qyqw),
      (_2qxqy + _2qzqw),     (1 - _2qxqx - _2qzqz), (_2qyqz - _2qxqw),
      (_2qxqz - _2qyqw),     (_2qyqz + _2qxqw),     (1 - _2qxqx - _2qyqy);

  if(posIndicator.getNumSubscribers() > 0){
    // tf visualisation (for debug)
    tf::Transform transform_msg;
    transform_msg.setOrigin(tf::Vector3(msgin->transform.translation.x/10.0, msgin->transform.translation.y/10.0, msgin->transform.translation.z/10.0));
    tf::Quaternion tf_q; tf_q.setX(msgin->transform.rotation.x); tf_q.setY(msgin->transform.rotation.y); tf_q.setZ(msgin->transform.rotation.z); tf_q.setW(msgin->transform.rotation.w);
    transform_msg.setRotation(tf_q);
    posPublisher.sendTransform(tf::StampedTransform(transform_msg, ros::Time::now(), "world", "imu_pos"));
    // marker
    vk::output_helper::publishHexacopterMarker(posIndicator, "imu_pos", "imus", ros::Time::now(), 1, 0, 0.3, Vector3d(0.,0.,1.));
  }

  // EKF imu measurement update
  Vector3d a; a << msgin->transform.translation.x, msgin->transform.translation.y, msgin->transform.translation.z;
  a *= 9.8;  // rescale the acceleration to m/ss

  // update objects
  mainFilter.StateUptSen(a, R, msgin->header.stamp.toSec());
  guard.imu_Cb(q);
  
  // publish measurments
  if (publisher.getNumSubscribers() > 0) {
	vi_ekf::teensyPilot output;
	output.qw     = (float)mainFilter.qVision[0];
	output.qx     = (float)mainFilter.qVision[1];
	output.qy     = (float)mainFilter.qVision[2];
	output.qz     = (float)mainFilter.qVision[3];
	output.px     = (float)mainFilter.state[0];
	output.py     = (float)mainFilter.state[1];
	output.pz     = (float)mainFilter.state[2];
	output.vx     = (float)mainFilter.state[3];
	output.vy     = (float)mainFilter.state[4];
	output.vz     = (float)mainFilter.state[5];
	output.bx     = (float)mainFilter.state[6];
	output.by     = (float)mainFilter.state[7];
	output.bz     = (float)mainFilter.state[8];
	output.lambda = (float)mainFilter.T;//state[9];//(float)(ros::Time::now().toSec()-946688000.0);//
	output.status = guard.commitOutput();
	publisher.publish(output);
  }
}


// MINE camera exposure Control
void ExposureCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin, 
		  tf::TransformBroadcaster& posPublisher, 
		  ros::Publisher& posIndicator, 
		  KalmanFilter10& mainFilter)
{
  if(mainFilter.reset_exposure == 0 && msgin->header.frame_id == "Exposure") {
    system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure false");  
    mainFilter.reset_exposure = 1;
  } else if (mainFilter.reset_exposure == 1 && msgin->header.frame_id == "Teensy") {
    system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure true");
    mainFilter.reset_exposure = 0;
  }	  
}



//____________________________
// Vision callback function
 void VISCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgin, 
		  tf::TransformBroadcaster& posPublisher, 
		  ros::Publisher& posIndicator, 
		  KalmanFilter10& mainFilter)
{ 
  // Copy message
  double cam_trans[4] = {0.0, msgin->pose.pose.position.x,    msgin->pose.pose.position.y,    msgin->pose.pose.position.z};
  mainFilter.qVision[0] = msgin->pose.pose.orientation.w;
  mainFilter.qVision[1] = msgin->pose.pose.orientation.x;
  mainFilter.qVision[2] = msgin->pose.pose.orientation.y;
  mainFilter.qVision[3] = msgin->pose.pose.orientation.z;
  
  // publish quaternion
  //if(ekfPublisher_quaternion.getNumSubscribers() > 0) ekfPublisher_quaternion.publish(msgin->pose.pose.orientation);
  
  // IMU camera position compensation
  double IMUpos_v[4] = {0.0, 0.0*mainFilter.state[9], 0.0*mainFilter.state[9], 0.06*mainFilter.state[9]}; // IMU position in vision frame {0.0, forward-x, left-y, up-z} in m (z = p*lambda) (from camera optical centre)
  double imu_trans[4];
  QuatRot(IMUpos_v, mainFilter.qVision, imu_trans); // compute compensation
  imu_trans[1] += cam_trans[1];          // add camera position measurements
  imu_trans[2] += cam_trans[2];
  imu_trans[3] += cam_trans[3];
  
  // estimate vision gravity
  //double g_s[4] = {0.0, 0.0, 0.0, -1.0}; // gravity in sensor frame
  //double g_v[4];
  //VPos2SPose(g_s, mainFilter.qVision, q_delayed, g_v); // here we use VPos2SPose() inversely to convert from sensor frame to vision frame
  //mainFilter.vision_gravity
  
  // Convert to gravity frame
  //Vector4d q_vec = mainFilter.q_bf.front();
  //double q_delayed[4] = {q_vec(0), q_vec(1), q_vec(2), q_vec(3)};
  //double pos_w[4];
  //VPos2SPose(imu_trans, q_delayed, mainFilter.qVision, pos_w); // note to use the delayed q, mainFilter.q_bf.front()

  // ######### EKF vision measurement update #######
  Vector3d p;	p << imu_trans[1], imu_trans[2], imu_trans[3];
  //Vector3d p;	p << pos_w[1], pos_w[2], pos_w[3];
  //std::cout<< ros::Time::now().toSec()-msgin->header.stamp.toSec()  <<std::endl;
  mainFilter.MeasureUptVis(p);
  
  
  // For visualisation
  if(posIndicator.getNumSubscribers() > 0){
    // /tf for visualisation
    tf::Transform transform_msg;
    tf::Quaternion tf_q;

    //tf visualisation (for debug) the visualisation should be the position output from vi_ekf fusion with correct orientation.
    transform_msg.setOrigin(tf::Vector3(mainFilter.state[0], mainFilter.state[1], mainFilter.state[2]));
    tf_q.setW(msgin->pose.pose.orientation.w);
    tf_q.setX(msgin->pose.pose.orientation.x);
    tf_q.setY(msgin->pose.pose.orientation.y);
    tf_q.setZ(msgin->pose.pose.orientation.z);

    // sent /tf message
    transform_msg.setRotation(tf_q);
    posPublisher.sendTransform(tf::StampedTransform(transform_msg, ros::Time::now(), "world", "svo_pos"));
    // marker
    vk::output_helper::publishHexacopterMarker(posIndicator, "svo_pos", "svos", ros::Time::now(), 1, 0, 0.3, Vector3d(0.,0.,1.));
  }
}


//______________________________
// Filter reset callback function
void resetCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin,
		   KalmanFilter10& mainFilter,
		   ValidationGuard& guard)
{
  mainFilter.toReset = true;
  mainFilter.resetLeft = mainFilter.reset_loop;
  guard.reset();
}


// _______________________________
// this is called when svo sends reset signal
void SVOresetCallback(const std_msgs::Bool::ConstPtr& msgin, 
		      KalmanFilter10& mainFilter,
		      ros::Publisher& resetPublisher,
		      double* q)
{
  geometry_msgs::TransformStamped msgout;
  msgout.transform.rotation.w = q[0]; msgout.transform.rotation.x = q[1]; msgout.transform.rotation.y = q[2]; msgout.transform.rotation.z = q[3];
  msgout.transform.translation.x = 0.0; msgout.transform.translation.y = 0.0; msgout.transform.translation.z = 0.0;
  
  resetPublisher.publish(msgout);
}





//#######################################################
int main(int argc, char **argv)
{
  // System Frequency
  const double frequency = 100.0;
  //double IMUfrequency = 180.0; // IMU updates at 91 Hz, which determines the EKF update rate.
  // Initiate kalman filter
  KalmanFilter10 kalmanFilter;//1.0/IMUfrequency); // use IMU update frequency period in sec 
  //kalmanFilter.reset_exposure = 0;
  //kalmanFilter.vision_gravity = {0.0, 0.0, 0.0, -1.0};  // initialise vision gravity vectory estimation !! need to correct the form
  double q[4] = {0.0, 0.0, 0.0, 0.0}; //this has to be here, because it is shared between two callbacks
  
  //_____________________________________
  // ROS node init
  ros::init(argc, argv, "ekfFusor");
  ros::NodeHandle n;

  // Publishers init
  tf::TransformBroadcaster imuPublisher;
  ros::Publisher           imuIndicator	      = n.advertise<visualization_msgs::Marker>("vi/imu/marker", 100);
  tf::TransformBroadcaster visPublisher;
  ros::Publisher           visIndicator	      = n.advertise<visualization_msgs::Marker>("vi/svo/marker", 100);

  //ros::Publisher         ekfPublisher_q     = n.advertise<geometry_msgs::Quaternion> ("/ekf/q", 2);
  //ros::Publisher	   ekfPublisher_p     = n.advertise<geometry_msgs::Vector3> ("/ekf/p", 2);
  //ros::Publisher	   ekfPublisher_v     = n.advertise<geometry_msgs::Vector3> ("/ekf/v", 2);
  //ros::Publisher	   ekfPublisher_b     = n.advertise<geometry_msgs::Vector3> ("/ekf/b", 2);
  //ros::Publisher	   ekfPublisher_lamda = n.advertise<std_msgs::Float64>      ("/ekf/lamda", 2);
  ros::Publisher         ekfPublisher = n.advertise<vi_ekf::teensyPilot> ("ekf/output", 1);
  
  ros::Publisher           resetPublisher     = n.advertise<geometry_msgs::TransformStamped>("Allreset",10);
  //ros::Publisher           mValidPublisher    = n.advertise<std_msgs::Int8>                 ("/ekf/safeout",100);
  ros::Publisher           iValidPublisher    = n.advertise<geometry_msgs::Vector3>         ("ekf/safeoutVec",100);

  // Initiate safe guard
  ValidationGuard guard(&kalmanFilter, &resetPublisher, &iValidPublisher);
  
  
  // Subscribers init
  ros::Subscriber imuSub = n.subscribe<geometry_msgs::TransformStamped>  ("teensy/imu", 1, boost::bind(IMUCallback,
										              _1,
											      boost::ref(imuPublisher),
											      boost::ref(imuIndicator),
											      boost::ref(kalmanFilter),
											      boost::ref(q),
											      boost::ref(guard),
											      boost::ref(ekfPublisher)));
											      
  ros::Subscriber exposureSub = n.subscribe<geometry_msgs::TransformStamped>  ("teensy/imu", 1, boost::bind(ExposureCallback,
										              _1,
											      boost::ref(imuPublisher),
											      boost::ref(imuIndicator),
											      boost::ref(kalmanFilter)));
											      
  ros::Subscriber visSub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("svo/pose",  1, boost::bind(VISCallback,
												    _1,
												    boost::ref(visPublisher),
												    boost::ref(visIndicator),
												    boost::ref(kalmanFilter)));
  ros::Subscriber resSub = n.subscribe<geometry_msgs::TransformStamped>("Allreset",  10, boost::bind(resetCallback,
  											_1,
  											boost::ref(kalmanFilter),
											boost::ref(guard)));
											
  ros::Subscriber resSub_ros = n.subscribe< std_msgs::Bool>("svo/usereset",  10, boost::bind(SVOresetCallback,
  											_1,
  											boost::ref(kalmanFilter),
											boost::ref(resetPublisher),
											boost::ref(q)));
  // for the validation guard
  guard.sub_svo_info_ = n.subscribe("svo/info",   1,  &ValidationGuard::svo_info_Cb, &guard);
  
  
  
  
  
  //________________________________________________
  // Spin
  ros::spin();
  /*ros::Rate loop_rate(frequency);
  int count = 0;
  while(ros::ok())
  { 
    // debug 
    //std::cout<< kalmanFilter.Pmat(0,0) <<", "<< kalmanFilter.Pmat(1,1) <<", "<< kalmanFilter.Pmat(2,2) <<", "<< kalmanFilter.Pmat(11,11) <<", "<< kalmanFilter.Pmat(12,12) <<std::endl;
  
    // Publish state
    // p, v, a, b
    geometry_msgs::Vector3 msgout;
    if(ekfPublisher_p.getNumSubscribers() > 0){
      msgout.x = kalmanFilter.state[0];  msgout.y = kalmanFilter.state[1];  msgout.z = kalmanFilter.state[2];  ekfPublisher_p.publish(msgout);}
    if(ekfPublisher_v.getNumSubscribers() > 0){// && count%2 == 0){
      msgout.x = kalmanFilter.state[3];  msgout.y = kalmanFilter.state[4];  msgout.z = kalmanFilter.state[5];  ekfPublisher_v.publish(msgout);}
    if(ekfPublisher_b.getNumSubscribers() > 0 && count%10 == 1){
      msgout.x = kalmanFilter.state[6];  msgout.y = kalmanFilter.state[7]; msgout.z = kalmanFilter.state[8]; ekfPublisher_b.publish(msgout);}
    if(ekfPublisher_lamda.getNumSubscribers() > 0 && count%10 == 3){
      std_msgs::Float64 lamdaout; lamdaout.data = (float)kalmanFilter.state[9]; ekfPublisher_lamda.publish(lamdaout);}
    
    //std::cout<< guard.commitOutput() <<std::endl;
    int a = guard.commitOutput();
    std_msgs::Int8 comitOut; comitOut.data = (int8_t)a;
    if(mValidPublisher.getNumSubscribers()>0) mValidPublisher.publish(comitOut);
    
    // House keeping
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  */

  return 0;
}
