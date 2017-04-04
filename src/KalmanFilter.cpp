#include <vi_ekf/KalmanFilter.h>
#include <sstream>
// a 10-state Kalman Filter to fuse monocular vision position with IMU
// Visual scale  can be recovered as well.

  //________________________
  // Constructor
  KalmanFilter10::KalmanFilter10(double n_s, double n_lamda, double n_b):
	delayed_imu_readings_(2), // vision has about 0.01ms delay, 180Hz imu update (0.0056), 0.01*180 = 1.8
	init_lambda(2.0),
	init_lambda_uncer(2.0), //1.5
        reset_exposure(0),
	T(1.0/180.0) // imu update time interval
  {
    // A
    Amat  = Matrix<double, 10, 10>::Identity();
    Amat.block<3,3>(0,3) = T * Matrix3d::Identity();
    
    // Q
    Matrix<double,7,7> Q = Matrix<double,7,7>::Zero();
    Q.block<3,3>(0,0) = n_s*n_s*Matrix3d::Identity();
    Q.block<3,3>(3,3) = n_b*n_b*Matrix3d::Identity();
    Q(6,6) = n_lamda*n_lamda;
    
    // W, WQWT
    Matrix<double, 10, 7> W = Matrix<double, 10,  7>::Zero();
    W.block<3,3>(3,0) = T * Matrix3d::Identity();
    W.block<4,4>(6,3) = Matrix<double,4,4>::Identity();
    WQWT  = W*Q*W.transpose();
    
    // Hv, KGain
    Hv    = Matrix<double,  3, 10>::Zero();
    KGain = Matrix<double, 10,  3>::Zero();
    
    // Reset kalman state
    resetLeft = 0;
    toReset = true;
    resetKalman();
    reset_loop = 15; // 20 continuous measurements before using the vision data
    resetLeft = reset_loop;
  }


  //________________________
  // Reset Kalman filter to initial state
  void KalmanFilter10::resetKalman(Vector3d pInit)
  {
    if (resetLeft==0) { // this allows to skip several vision measurements at initailisation
	// for robust reset
        toReset = false;
        resetLeft = reset_loop;
	
    } else {
	// for robust reset
	--resetLeft;
	
	// Init State Vector
        state = Matrix<double, 10,  1>::Zero();
        state.head<3>() = pInit;
        state(9) = init_lambda;

        // Init State/process Propability Matrix (assumes large initial uncertainty)
	Matrix<double, 10, 1> vecTwo; vecTwo << 1.0,1.0,1.0, 1.0,1.0,1.0, 0.5,0.5,0.5, init_lambda_uncer; //5 2 0.1 3
        Pmat  = vecTwo * vecTwo.transpose();
	
	// clear imu buffers
	accel_bf.clear();
	Rot_bf.clear();
	q_bf.clear();
	
	// reset lambda init count
	lambda_ini_count = 10;
	
	// estimation error
	estError_ << 100.0, 100.0, 100.0;
    }
  }


  //_________________________
  // State process update, when new sensor measurement is available
  void KalmanFilter10::StateUptSen(const Vector3d& accel, const Matrix3d& Rot, double msgTime)
  {
    // insert to buffer
    Rot_bf.push_back(Rot);
    accel_bf.push_back(accel);
 
    if (Rot_bf.size() == (delayed_imu_readings_+1)) {// num of delayed measurements + 1
      //Update time interval
      double dt = msgTime - imu_timer_;
      if (dt < 0.05 && dt > 0.0) {
        T = 199.0/200.0 * T + (dt / 200.0); // update the time interval (average over 200 msgs)
      }
      imu_timer_ = msgTime;

      // Update A
      Amat.block<3,3>(0,3) = T * Matrix3d::Identity();
      Amat.block<3,3>(3,6) = T * Rot_bf.front();
    
      // state Means (xnew = f(x))
      state.block<3,1>(3,0) += Amat.block<3,3>(3,6)*state.block<3,1>(6,0) + T*accel_bf.front();
      state.block<3,1>(0,0) += T * state.block<3,1>(3,0);
    
      //_____________________________
      // state Probability distribution
      Pmat = Amat * Pmat * Amat.transpose() + WQWT;
    
      // delet old buffer elements
      Rot_bf.pop_front();
      accel_bf.pop_front();
    }
  }




  //_________________________
  // Filter vision measurement update
  void KalmanFilter10::MeasureUptVis(const Vector3d& z, double n_v)
  {
    if (toReset) {
      resetKalman(z/init_lambda);
    } else if (Rot_bf.size() == delayed_imu_readings_) { // ensure imu data comes first
      Hv.block<3,3>(0,0) = state(9) * Matrix3d::Identity();
      Hv.col(9) = state.head<3>();
      VRVT  = n_v*n_v*Matrix3d::Identity();

      Matrix3d temp = Hv * Pmat * Hv.transpose() + VRVT;
      KGain  = Pmat * Hv.transpose() * temp.inverse();
      estError_ = z - (state(9)*state.head<3>());
      state_correction = KGain * estError_;
      state += state_correction;
      Pmat   = (Matrix<double,10,10>::Identity() - (KGain * Hv)) * Pmat;
      
      if (lambda_ini_count >0){
        state(9)  = init_lambda;
	Pmat(9,9) = init_lambda_uncer * init_lambda_uncer;
        lambda_ini_count--;
      }
    }
  }




