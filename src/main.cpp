//general utilities
#include <cmath>
#include <math.h>
#include <fstream>
#include <string>
#include <time.h>
#include <stdlib.h>
// #include <flann/flann.h>
// #include <flann/io/hdf5.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>


// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/JointState.h>



// utilities
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>



#include "reactiveGraspDetection.h"


void signalHandler( int signum )
{
	ros::shutdown();
}



int main(int argc, char** argv)
{	
	ros::init(argc, argv, "reactive_grasp", ros::init_options::NoSigintHandler);
  	ros::NodeHandle n;

  	// register signal SIGINT (CTRL+C) and signal handler  
    signal(SIGINT, signalHandler);  

	ReactiveGraspDetection demo;

	return 0;
}