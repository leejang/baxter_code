cmake_minimum_required(VERSION 2.8.3)
project(baxter_learning_from_egocentric_video)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  find_object_2d
  tf
  image_geometry
#  moveit_core
#  moveit_msgs
#  moveit_ros_planning
#  moveit_ros_planning_interface
  geometry_msgs
  cmake_modules
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Target.msg
  TargetJoints.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_learning_from_egocentric_video
#  CATKIN_DEPENDS baxter_core_msgs, cv_bridge, image_transport roscpp, sensor_msgs, std_msgs,
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(baxter_learning_from_egocentric_video
#   src/${PROJECT_NAME}/baxter_learning_from_egocentric_video.cpp
# )

# set header ans source files
# Object manipulation
set(MANIPULATION_SOURCE_FILES
  src/object_manipulation.cpp
  src/baxter_controller.cpp
  src/baxter_cameras.cpp
  src/object_detector.cpp
)

set(MANIPULATION_HEADER_FILES
  src/baxter_controller.h
  src/baxter_cameras.h
  src/object_detector.h
)

# Kinect Viewer
set(KINECT_VIEWER_SOURCE_FILES
  src/kinect_viewer.cpp
  src/kinect_viewer_node.cpp
)

set(KINECT_VIEWER_HEADER_FILES
  src/kinect_viewer.h
)

# Hand detector
set(HAND_DETECTOR_SOURCE_FILES
  src/hand_detector.cpp
  src/hand_detection_node.cpp
)

set(HAND_DETECTOR_HEADER_FILES
  src/hand_detector.h
)

# Skeleton tracker
set(SKELETON_TRACKER_SOURCE_FILES
  src/skeleton_tracker.cpp
  src/skeleton_tracker_node.cpp
)

set(SKELETON_TRACKER_HEADER_FILES
  src/skeleton_tracker.h
)
# Baxter motion (demo code)
set(BAXTER_MOTION_SOURCE_FILES
  src/baxter_motion.cpp
  src/baxter_controller.cpp
)

set(BAXTER_MOTION_HEADER_FILES
  src/baxter_motion.h
  src/baxter_controller.h
)

# Baxter motion (demo code)
set(BAXTER_MOTION_2_SOURCE_FILES
  src/baxter_motion2.cpp
  src/baxter_controller.cpp
)

set(BAXTER_MOTION_2_HEADER_FILES
  src/baxter_motion2.h
  src/baxter_controller.h
)

# Baxter motion (demo code)
set(BAXTER_MOVEIT_MOTION_SOURCE_FILES
  src/baxter_moveit_motion.cpp
  src/baxter_moveit_controller.cpp
)

set(BAXTER_MOVEIT_MOTION_HEADER_FILES
  src/baxter_moveit_motion.h
  src/baxter_moveit_controller.h
)

# Training
set(TRAIN_SOURCE_FILES
  src/train_representation.cpp
  src/train_representation_node.cpp
)

set(TRAIN_HEADER_FILES
  src/train_representation.h
)

# Test: future prediction
set(FUTURE_PREDICTION_SOURCE_FILES
  src/future_prediction.cpp
  src/future_prediction_node.cpp
  src/baxter_controller.cpp
  src/baxter_moveit_controller.cpp
)

set(FUTURE_PREDICTION_HEADER_FILES
  src/future_prediction.h
  src/baxter_controller.cpp
  src/baxter_moveit_controller.h
)

# Making Robot Training Data
set(MAKING_ROBOT_TRAIN_D_SOURCE_FILES
  src/making_robot_training_data.cpp
  src/making_robot_training_data_node.cpp
)

set(MAKING_ROBOT_TRAIN_D_HEADER_FILES
  src/making_robot_training_data.h
)

# For OpenCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# For Caffe
find_package(Caffe)
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS}) # ex. -DCPU_ONLY

# For Window Proposals
set(MATLAB_INCLUDE_DIR
  /usr/local/MATLAB/R2015a/extern/include
)

set(MATLAB_LINK_DIR
  /usr/local/MATLAB/R2015a/bin/glnxa64
)

set(MATLAB_LIBRARIES
  /usr/local/MATLAB/R2015a/bin/glnxa64/libeng.so
  /usr/local/MATLAB/R2015a/bin/glnxa64/libmat.so
  /usr/local/MATLAB/R2015a/bin/glnxa64/libmex.so
  /usr/local/MATLAB/R2015a/bin/glnxa64/libut.so
)

include_directories(${MATLAB_INCLUDE_DIR})
#link_directories(${MATLAB_LINK_DIR})

##########################################
# To used skeltrack lib instead of openni_tracker.
# Currently, we use openni_tracker.
if(FALSE)
# For Skeleton Tracking
set(SKEL_TRACK_INCLUDE_DIR
  /home/leejang/local/include/skeltrack-0.1
  /usr/include/glib-2.0
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
)

set(SKEL_TRACK_LINK_DIR
  /home/leejang/local/lib
)

set(SKEL_TRACK_LIBRARIES
  skeltrack-0.1
)
include_directories(${SKEL_TRACK_INCLUDE_DIR})
link_directories(${SKEL_TRACK_LINK_DIR})
endif()
###########################################

## Declare a C++ executable
add_executable(object_manipulation ${MANIPULATION_SOURCE_FILES} ${MANIPULATION_HEADER_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(object_manipulation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(object_manipulation ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(kinect_viewer ${KINECT_VIEWER_SOURCE_FILES} ${KINECT_VIEWER_HEADER_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(kinect_viewer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(kinect_viewer ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(hand_detector ${HAND_DETECTOR_SOURCE_FILES} ${HAND_DETECTOR_HEADER_FILES})
## Specify libraries to link a library or executable target against
#target_link_libraries(hand_detector ${catkin_LIBRARIES} ${Caffe_LIBRARIES} ${MATLAB_LIBRARIES})
## Add cmake target dependencies of the executable/library
#add_dependencies(hand_detector ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(skeleton_tracker ${SKELETON_TRACKER_SOURCE_FILES} ${SKELETON_TRACKER_HEADER_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(skeleton_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(skeleton_tracker ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(baxter_motion ${BAXTER_MOTION_SOURCE_FILES} ${BAXTER_MOTION_HEADER_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(baxter_motion ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(baxter_motion ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(baxter_motion2 ${BAXTER_MOTION_2_SOURCE_FILES} ${BAXTER_MOTION_2_HEADER_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(baxter_motion2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(baxter_motion2 ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(baxter_moveit_motion ${BAXTER_MOVEIT_MOTION_SOURCE_FILES} ${BAXTER_MOVEIT_MOTION_HEADER_FILES})
## Specify libraries to link a library or executable target against
#target_link_libraries(baxter_moveit_motion ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
#add_dependencies(baxter_moveit_motion ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(train_representation ${TRAIN_SOURCE_FILES} ${TRAIN_HEADER_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(train_representation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(train_representation ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(future_prediction ${FUTURE_PREDICTION_SOURCE_FILES} ${FUTURE_PREDICTION_HEADER_FILES})
## Specify libraries to link a library or executable target against
#target_link_libraries(future_prediction ${catkin_LIBRARIES} ${Caffe_LIBRARIES})
## Add cmake target dependencies of the executable/library
#add_dependencies(future_prediction ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(publish_train_images src/publish_train_images_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(publish_train_images ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(publish_train_images ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(record_visual_odometry src/record_visual_odometry_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(record_visual_odometry ${catkin_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(record_visual_odometry ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(make_robot_train_d ${MAKING_ROBOT_TRAIN_D_SOURCE_FILES} ${MAKING_ROBOT_TRAIN_D_FILES})
## Specify libraries to link a library or executable target against
#target_link_libraries(make_robot_train_d ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
#add_dependencies(make_robot_train_d ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baxter_learning_from_egocentric_video.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
