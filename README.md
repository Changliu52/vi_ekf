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

Some test quadrotor flight running the packages in the onboard Odroid U3 and Odroid XU4 has been recorded in Youtube:
https://youtu.be/1TrFu_Ctv0s?list=PLIAB0d1xJQQ6Q7A-8muH41tBeMel9w_V7
https://youtu.be/yl_cu3SLcu0?list=PLIAB0d1xJQQ6Q7A-8muH41tBeMel9w_V7
https://youtu.be/F8lnu_FbV44?list=PLIAB0d1xJQQ6Q7A-8muH41tBeMel9w_V7
https://youtu.be/IICqtjW25MQ?list=PLIAB0d1xJQQ6Q7A-8muH41tBeMel9w_V7
https://youtu.be/WMEQ7CppB0E?list=PLIAB0d1xJQQ6Q7A-8muH41tBeMel9w_V7
https://youtu.be/R7FX-kKlSRg?list=PLIAB0d1xJQQ6Q7A-8muH41tBeMel9w_V7

Copyright (c) 2016, Chang Liu Chang.liu52@icloud.com. for commercial use please contact him.

All rights reserved.

BSD3 license: see LICENSE file

***How to use it***

** hardware setup **

It was tested on Odroid U3 and Odroid XU4.
The camera used was global shutter monocular ueye camera UI-1221LE https://en.ids-imaging.com/store/produkte/kameras/ui-1221le.html
The calibrated fisheye lens is https://www.lensation.de/product/BM2118V2C/ with IR cut, or https://www.lensation.de/product/BM2118V2/ without IR cut.

It only work in conjunction with the Unicorn autopilot (teensy-based): https://github.com/Changliu52/UnicornTeensyDrone.
For more detailed setup follow the link:

** ROS setup **

Tested environment: 
Lubuntu 14.04
Lubuntu 16.04
ROS Indigo
ROS Kinetic

This is a ROS catkin_make package, which is meant to be used in conjunction with the following ROS packages:
1. Customised SVO with customised vikit: Please follow the link https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-General-for-ARM-processors and https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-ROS for installation. But remember to clone the customised SVO in https://github.com/Changliu52/rpg_svo, and customised vikit in https://github.com/Changliu52/rpg_vikit
2. Customised Ueye camera dirver: https://github.com/Changliu52/ueye_cam (make sure you install the system driver for ueye following the link https://en.ids-imaging.com/download-ueye-emb-hardfloat.html)
3. Customised Rosserial: https://github.com/Changliu52/rosserial

Then 'catkin_make' the 

