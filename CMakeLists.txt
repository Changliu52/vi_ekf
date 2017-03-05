cmake_minimum_required(VERSION 2.8.3)
project(vi_ekf)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  svo_msgs
  genmsg
  geometry_msgs
  tf
  vikit_ros
)

## For eigen
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)

## messages
############################################
find_package(message_generation)

 add_message_files(
   FILES
   teensyPilot.msg
   #teensyPilotLong.msg
 )
 
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vi_ekf
  CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

## Create ekf library
#################################
# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/HelperFunc.cpp
  src/KalmanFilter.cpp
  src/MyMatMath.cpp
  src/ValidationGuard.cpp)
ADD_LIBRARY(ekf ${SOURCEFILES})
TARGET_LINK_LIBRARIES(ekf ${catkin_LIBRARIES})


## create vi_ekf executable
#############################################
add_executable(vi_ekf src/main.cpp)
target_link_libraries(vi_ekf ${catkin_LIBRARIES} ekf)
