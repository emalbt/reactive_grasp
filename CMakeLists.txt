cmake_minimum_required(VERSION 2.8.3)
project(reactive_grasp)


find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  cv_bridge
  image_transport 
  std_srvs
  sensor_msgs
  message_generation
  tf
  tf_conversions
  cmake_modules  #need for eigen
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  sensor_msgs
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  roscpp 
  tf_conversions 
  message_generation 
  std_msgs
  geometry_msgs 
  sensor_msgs
  lwr_controllers
)



link_directories(${catkin_LIBRARY_DIRS})


add_definitions(-std=c++11 --all-warnings -O3) 

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
  controller_interface
  control_msgs
  control_toolbox
  realtime_tools
  urdf
  forward_command_controller
  kdl_parser
  message_runtime
  tf_conversions
  lwr_hw
  lwr_controllers
  calibration
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS} include)



# Declare a C++ executable
add_executable(reactive_grasp_node src/main.cpp src/IMUboardAPI.cpp src/reactiveGraspDetection.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(reactive_grasp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Specify libraries to link a library or executable target against
target_link_libraries(reactive_grasp_node ${catkin_LIBRARIES} )
