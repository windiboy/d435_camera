cmake_minimum_required(VERSION 2.8.3)
project(d435_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
)
find_package(OpenCV 3.4.2)

set(REALSENSE_INCLUDE /opt/ros/melodic/include/)
set(EIGEN_INCLUDE /usr/include/eigen3/)
set(REALSENSE_SRC /opt/ros/melodic/lib/librealsense2.so)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES d435_camera
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${REALSENSE_INCLUDE}
  ${EIGEN_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


 add_executable(camera src/camera.cpp )
 add_dependencies(camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(camera ${catkin_LIBRARIES} ${REALSENSE_SRC} ${OpenCV_LIBS})

add_executable(rs_imshow src/rs_imshow.cpp)
add_dependencies(rs_imshow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rs_imshow ${catkin_LIBRARIES} ${REALSENSE_SRC} ${OpenCV_LIBS})

#############
## Install ##
#############
