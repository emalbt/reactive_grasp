/*
	reactiveGraspDetection
	
    Copyright (C) 2016	Emanuele Luberto (emanuele.luberto@gmail.com)
	
	Author affiliation:
		Emanuele Luberto - Research Center “E.Piaggio”,School of Engineering,University of Pisa 
							from 01-april-2016 to current


    reactiveGraspDetection is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or 
	any later version.

    reactiveGraspDetection is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FORz A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with reactiveGraspDetection. If not, see <http://www.gnu.org/licenses/>.
*/
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <boost/chrono.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio/serial_port.hpp> 
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>


// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/JointState.h>


#include <IMUboardAPI.h>

#define _RECORD_ALL_DATA_ 1

struct Sensor
{
    double x;
    double y;
    double z;
};

struct IMUData
{
    // Acceleration data
    std::vector<Sensor> acc;
    
    // Gyroscope data
    std::vector<Sensor> gyro;  

};




class ReactiveGraspDetection
{
public:
	/*		Contructor         */
    ReactiveGraspDetection();

    /*		Destructor         */
    ~ReactiveGraspDetection();

    //===============================================================================     ProcessData
    // void processData(ros::Duration acquisition_time);
    void processData();


private:
    // Ros Varaibles
    ros::NodeHandle node_handle_;
    ros::Time start_time_;

   
    

    //===============================================================================     Append to Log Files
    void appendToLogFile(std::ofstream *log_file, std::vector<Sensor> data);

    //===============================================================================     Filter Data
    void filterData();

    //===============================================================================     Init GLove Communication
    void initGloveCommunication();
    IMUboardAPI glove_;
 
    //===============================================================================     Init Parameters
    void initParameters();
    std::vector<IMUData> raw_data_; //example: raw_data_[#sample].acc[#imu].x   raw_data_[#sample].gyro[#imu].x
    std::vector<IMUData> filtered_data_;    

    int samples_;
    int nIMU_;
    // log files
    std::string log_file_name_raw_;
    std::string log_file_name_filt_;
    std::string log_file_name_map_;
    std::string log_file_name_gyro_;
    std::string log_file_name_ACC_raw_;
    std::string log_file_name_ACC_filt_;
    std::string log_file_base_path_;
    std::string date_time_;
    std::ofstream log_file_accel_raw_;
    std::ofstream log_file_accel_filt_;
    std::ofstream log_file_accel_map_;
    std::ofstream log_file_gyro_raw_;
    std::ofstream log_file_ACC_raw_;
    std::ofstream log_file_ACC_filt_;

   
    //===============================================================================     Update Raw Data
    void updateRawData();
    float dt_;

    //===============================================================================     Update Log Files
    void updateLogFiles();

};