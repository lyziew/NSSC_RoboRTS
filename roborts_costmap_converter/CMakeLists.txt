cmake_minimum_required(VERSION 2.8.3)
project(roborts_costmap_converter)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
  roborts_msgs
  roborts_costmap
  roborts_common
  dynamic_reconfigure
  pluginlib
  cv_bridge
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
###set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
###find_package(Eigen3 REQUIRED)
###set(EXTERNAL_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## C++11 support
set(CMAKE_CXX_STANDARD 14)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/dynamic_reconfigure/CostmapToDynamicObstacles.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(roborts_costmap_converter 
  src/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.cpp
  src/costmap_to_dynamic_obstacles/background_subtractor.cpp
  src/costmap_to_dynamic_obstacles/blob_detector.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/Ctracker.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/Kalman.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/HungarianAlg.cpp
)
target_link_libraries(roborts_costmap_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_compile_features(roborts_costmap_converter PUBLIC cxx_nullptr cxx_range_for)

# Dynamic reconfigure: make sure configure headers raare built before any node using them
add_dependencies(roborts_costmap_converter ${PROJECT_NAME}_gencfg)

## Declare a cpp executable
add_executable(standalone_converter src/costmap_converter_node.cpp)
target_link_libraries(standalone_converter ${catkin_LIBRARIES} ${GLOG_LIBRARY} roborts_costmap)
add_dependencies(standalone_converter roborts_costmap_converter)