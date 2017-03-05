***VISUAL-INERTIAL SENSOR FUSION FRAMEWORK***

**DISCLAMER:**

This ros package was developed to be used as a sensor fusion framework for the state estimation of robots, drones or VR/AR headsets.
It loosely couples a 6Dof SLAM (Simultaneous localisation and mapping) algorithm with 6-axis IMU (Inertial measurement unit, including a gyroscope and an accelerometer).
More specifically this sensor fusion framework has been tested directly compatible with:
	a slightly modified SVO (Semidirectly visual odometry) algorithm, (http://github.com/Changliu52/rpg_svo)
	in conjunction with an Teensy based IMU system.

This project was created within an academic research environment, thus should be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the code, so please adjust expectations accordingly.

Related publications:
	Computationally efficient visual-inertial sensor fusion for Global Positioning System-denied navigation on a small quadrotor (http://doi.org/10.1177/1687814016640996)

Copyright (c) 2016, Chang Liu

All rights reserved.

BSD3 license: see LICENSE file
