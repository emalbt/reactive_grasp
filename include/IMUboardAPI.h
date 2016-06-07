/*
    IMUboardAPI - IMU GLOVE LIBRARY
    
    Copyright (C) 2016  Emanuele Luberto (emanuele.luberto@gmail.com)
    
    Author affiliation:
        Emanuele Luberto - Research Center “E.Piaggio”,School of Engineering,University of Pisa 
                            from 01-april-2016 to current


    IMUboardAPI is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or 
    any later version.

    IMUboardAPI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FORz A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SHOG. If not, see <http://www.gnu.org/licenses/>.
*/
#include <string>
#include <sstream>
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


#define _DEBUG_DATA_BUFFER_ 0
#define _DEBUG_REAL_DATA_   0

// # step for compute offset angles
#define _OFFSET_STEP_      100


// sgn function
#define sgn(s) (s>=0.0?1:-1)


/* Variables useful to PSoC communication */
struct psocVariables
{
    // PSoC port
    char*   port; 
    // baudRate 
    int     baudRate;
    // # of IMU connected
    int     nIMU;
    // buffer size for one IMU
    int     byteIMU;   
};



class IMUboardAPI
{
public:
    /*          Contructor         */
    IMUboardAPI();

    /*          Destructor         */
    ~IMUboardAPI();
    
    // public variables
    Eigen::MatrixXd Acc_;
    Eigen::MatrixXd Acc_Old_;
    Eigen::MatrixXd Gyro_;
    Eigen::MatrixXd Gyro_Old_;
    Eigen::MatrixXd Gyro_Bias_;


    //================================================================     InitPSoC
    void initPSoC();
    psocVariables p_;
    
    //================================================================     StopPSoC
    void stopPSoC();
    
    //================================================================     ReadPSoC
    void readPSoC();



private:

    boost::asio::serial_port* serialPort_;
    boost::asio::io_service ioService_;
    int sizeBuffer_;
    uint8_t* dataBuffer_; 


    //================================================================     Useful Functions 
    Eigen::Matrix3d rotX( float x );
    Eigen::Matrix3d rotY( float y );
    Eigen::Matrix3d rotZ( float z );
    Eigen::Matrix3d rot( float x, float y, float z );
    Eigen::Matrix3d skew( Eigen::Vector3d a );
    Eigen::Matrix3d Quat2Rot( Eigen::Vector4d Q_in );
    Eigen::Vector4d ConjQ( Eigen::Vector4d Q_in );
    Eigen::Vector4d QxQ( Eigen::Vector4d Q_1, Eigen::Vector4d Q_2 );
    Eigen::Vector4d Rot2Quat( Eigen::Matrix3d R_in );
    Eigen::Vector3d Quat2Angle( Eigen::Vector4d Q_in );
    Eigen::Vector3d Rot2Angle( Eigen::Matrix3d R_in );
};

































