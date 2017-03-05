#ifndef __KalmanFilter_H__
#define __KalmanFilter_H__
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>


// a 10-state Kalman Filter to fuse monocular vision position with IMU
// Visual scale  can be recovered as well.
using namespace Eigen;

class KalmanFilter10
{
  public:
  // reset flag
  bool toReset;
  int resetLeft;     // counter for how many time to reset (essential skip several vision measurements at the beginning.
  int reset_loop;    // user defined number of time to reset
  
  // Main state variable
  Matrix<double, 10,  1> state;
  Matrix<double, 10,  1> state_correction;
  Matrix<double, 10, 10> Pmat;
  Vector3d estError_;
  double T;           // time update period from IMU
  
  // buffer for timing (to delay accel reading) (FOR EXTERNAL USAGE)
  std::list<Vector3d> accel_bf;
  std::list<Matrix3d> Rot_bf;
  std::list<Vector4d> q_bf;
  double imu_timer_;
  const int delayed_imu_readings_;
  double qVision[4];
  double vision_gravity[4];
  bool reset_exposure;
  
  //________________________
  // Constructor
  KalmanFilter10(double acelNoise = 0.2, double lamdaNoise = 0.0, double bNoise = 0.0);// a=0.1265,lam=0.0,b=0.0001 sigma=0.1265 m/ss: power spectral density = 400 ug/(hz^0.5) @1kHz
	//			    ^ 0.1265 we are loosing this considering the vibration
  //________________________
  // Reset Kalman filter to initial state
  void resetKalman(Vector3d pInit = Matrix<double, 3,  1>::Zero());

  //_________________________
  // State process update, when new sensor measurement is available
  void StateUptSen(const Vector3d& accel, const Matrix3d& Rot, double msgTime);

  //_________________________
  // Filter vision measurement update
  void MeasureUptVis(const Vector3d& z, double n_v = 0.0001);//0.0001  0.3mm accuracy when the map scale is 1


  //_________________________
  //########################
  private:
  Matrix<double, 10, 10> Amat;
  Matrix<double,  3, 10> Hv;

  Matrix<double, 10, 10> WQWT;
  Matrix<double,  3,  3> VRVT;

  Matrix<double, 10,  3> KGain;


  // system noise assumption (std deviation)
  double n_a;         // system accel noise
  double n_lamda;     // system lamda noise
  double n_b;         // system bias  noise
  // for initialisation
  double init_lambda;		// initial lambda value
  double init_lambda_uncer;	// initial lambda uncertainty sigma
  // smooth init lambda
  int lambda_ini_count;         // number of measurements skipped for ini lambda estimation
};


#endif

