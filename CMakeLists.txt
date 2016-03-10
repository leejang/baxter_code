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
# Controller
set(MANIPULATION_SOURCE_FILES
  src/object_manipulation.cpp
  src/baxter_controller.cpp
  src/baxter_cameras.cpp
)

set(MANIPULATION_HEADER_FILES
  src/baxter_controller.h
  src/baxter_cameras.h
)

# For OpenCV
find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(object_manipulation ${MANIPULATION_SOURCE_FILES} ${MANIPULATION_HEADER_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(object_manipulation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Add cmake target dependencies of the executable/library
add_dependencies(object_manipulation ${catkin_EXPORTED_TARGETS})

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
