/*
	IMUboardAPI
	
    Copyright (C) 2016	Emanuele Luberto (emanuele.luberto@gmail.com)
	
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
    along with IMUboardAPI. If not, see <http://www.gnu.org/licenses/>.
*/

#include "IMUboardAPI.h"

// ============================================================================ Constructor#2
IMUboardAPI::IMUboardAPI()
{
	/********************
	*					*
	*	CODE VERSION	*
	*					*
	********************/
	/*
		Release date: 20-apr-2016
		Version: vMajorVersion.MinorVersion.Patcsh
		Release Version: v0.3.20
	*/
	time_t rawtime;
	struct tm * timeinfo;
	int majorVersion;
	int minorVersion;
	int patchVersion;
	std::stringstream ssVersion;
	
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	//tm_year (int)	--> years since 1900
	//tm_mon (int)	--> months since January (0-11)
	//tm_mday (int)	--> day of the month (1-31)
	majorVersion = (1900 + timeinfo->tm_year) - 2016;
	minorVersion = timeinfo->tm_mon;
	patchVersion = timeinfo->tm_mday;
	
	ssVersion.str("");
	ssVersion<<"v"<<majorVersion<<"."<<minorVersion<<"."<<patchVersion<<"\n";
	
	std::cout<<"\nWelcome\n\tIMUboardAPI "<<ssVersion.str()<<"\n";
}


// =============================================================================================
//																					 Desctructor
// =============================================================================================
IMUboardAPI::~IMUboardAPI()
{
	//nothing to be done
	std::cout << "\n\n\n\r\033[31m\033[1mSHUTDOWN IMUboardAPI\033[0m \r\n\n\n";
}




/**********************************************************************************************/
/*																					  		  */
/*																					  		  */
/*					                  		 FUNCTIONS                                        */
/*																					    	  */
/*																					  		  */
/**********************************************************************************************/


// =============================================================================================
//																						InitPSoC
// =============================================================================================
void IMUboardAPI::initPSoC()
{

	// init public variables
	Acc_.resize(p_.nIMU, 3);
	Acc_Old_.resize(p_.nIMU,3);
	Gyro_.resize(p_.nIMU, 3);
	Gyro_Old_.resize(p_.nIMU,3);
	Gyro_Bias_.resize(p_.nIMU,3);
	Gyro_Bias_.setZero();

	


	// Init Communication
	usleep(20000);
	sizeBuffer_ = p_.nIMU*p_.byteIMU;
	dataBuffer_ = (new uint8_t[sizeBuffer_]);

	serialPort_ = (new boost::asio::serial_port(ioService_));
	serialPort_->close();
	serialPort_->open(p_.port);
	serialPort_->set_option(boost::asio::serial_port_base::baud_rate(p_.baudRate));
	serialPort_->set_option(boost::asio::serial_port_base::character_size(8));
	serialPort_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	serialPort_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	serialPort_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

	tcflush(serialPort_->lowest_layer().native_handle(), TCIOFLUSH);

	std::cout<< "\r\nCOMMUNICATION ESTABLISHED WITH THE PSOC5 \n";

	std::cout << "p_.port: " << p_.port <<std::endl;
	std::cout << "p_.baudRate: " << p_.baudRate<<std::endl;
	std::cout << "p_.nIMU: " << p_.nIMU <<std::endl;
	std::cout << "p_.byteIMU: " << p_.byteIMU <<std::endl;

	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('r'),1));
	std::cout<< "\r\nBLINKING LED?  \n";


	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('?'),1));
	// boost::asio::streambuf dataStreamBuf;
	// boost::asio::read_until(*serialPort_,dataStreamBuf,'\n');		
	// std::string strData(boost::asio::buffer_cast<const char*>(dataStreamBuf.data()));
	// std::vector<std::string> strSensorData;//sensor data in string format
	// boost::split(strSensorData, strData, boost::is_any_of("\n"));
	// std::cout << "Firmware Version: " << strData << std::endl;
    getchar();

	readPSoC();
	Acc_Old_ = Acc_;
	Gyro_Bias_ = Gyro_;
}


// =============================================================================================
//																						InitPSoC
// =============================================================================================
void IMUboardAPI::stopPSoC()
{
	std::cout<< "\r\n\nSTOP COMMUNICATION\n";
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('r'),1));
	serialPort_->close();
}


// =============================================================================================
//																						ReadPsoC
// =============================================================================================
void IMUboardAPI::readPSoC()
{
	// IMU glove
	// read serial port buffer
	boost::asio::write(*serialPort_,boost::asio::buffer(new char('<'),1));
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('\r'),1));
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('\n'),1));

	// IMUKUKA
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('?'),1));

	boost::asio::read(*serialPort_, boost::asio::buffer(dataBuffer_, sizeBuffer_));

	if (_DEBUG_DATA_BUFFER_)
	{
		for(int i=0; i<sizeBuffer_; i++ )
		{
		    std::cout <<i<<" " << (double) dataBuffer_[i] << "\r\n";  
		}		 	
	}

	// record data in the Matrix
	int16_t		acc [p_.nIMU][3];
	int16_t		gyro[p_.nIMU][3];

	int 		rateAcc = 2;
	float 		scaleAccFactor  = 0.000061037 * rateAcc;  // *2  = form 2g to 4g  
	int 		rateGyro = 8;
	float 		scaleGyroFactor = 0.007629627 * rateGyro; // " *8 " = from 250°/s to 2000°/s;

	for(int i=0; i<p_.nIMU; i++)
	{
		acc[i][0] = (dataBuffer_[1+(p_.byteIMU*i)]<<8 | dataBuffer_[2+(p_.byteIMU*i)]);
        acc[i][1] = (dataBuffer_[3+(p_.byteIMU*i)]<<8 | dataBuffer_[4+(p_.byteIMU*i)]);
        acc[i][2] = (dataBuffer_[5+(p_.byteIMU*i)]<<8 | dataBuffer_[6+(p_.byteIMU*i)]);


        gyro[i][0] = (dataBuffer_[7 +(p_.byteIMU*i)]<<8 | dataBuffer_[8 +(p_.byteIMU*i)]);
        gyro[i][1] = (dataBuffer_[9 +(p_.byteIMU*i)]<<8 | dataBuffer_[10+(p_.byteIMU*i)]);
        gyro[i][2] = (dataBuffer_[11+(p_.byteIMU*i)]<<8 | dataBuffer_[12+(p_.byteIMU*i)]);
        
	
        Acc_(i,0) = (double)  acc[i][0] * scaleAccFactor;
        Acc_(i,1) = (double)  acc[i][1] * scaleAccFactor;
        Acc_(i,2) = (double)  acc[i][2] * scaleAccFactor;

        Gyro_(i,0) = (double) gyro[i][0] * scaleGyroFactor;
        Gyro_(i,1) = (double) gyro[i][1] * scaleGyroFactor;
        Gyro_(i,2) = (double) gyro[i][2] * scaleGyroFactor;

        Gyro_(i,0) -= Gyro_Bias_(i,0);
        Gyro_(i,1) -= Gyro_Bias_(i,1);
        Gyro_(i,2) -= Gyro_Bias_(i,2);

        if(_DEBUG_REAL_DATA_)
        {
        	std::cout << i <<"  acc:\t\t" << Acc_(i,0) << "\t" << Acc_(i,1) << "\t" << Acc_(i,2)<<"\r\n"; 
        	std::cout << " " <<"  gyro:\t" << Gyro_(i,0) << "\t" << Gyro_(i,1) << "\t" << Gyro_(i,2)<<"\r\n";
        }
	}

	if(_DEBUG_REAL_DATA_)
	{
		std::cout <<"\n\n";
	}
}




// ============================================================================================= 
//																				Useful Functions
// =============================================================================================
Eigen::Matrix3d IMUboardAPI::rotX(float x)
{
	Eigen::Matrix3d Rx;
	x = x *(M_PI/180); // From deg to Rad
	Rx << 1 , 0, 0,
		  0, cos(x), -sin(x),
		  0, sin(x), cos(x);
	return Rx;
}
Eigen::Matrix3d IMUboardAPI::rotY(float y)
{
	Eigen::Matrix3d Ry;
	y = y *(M_PI/180); // From deg to Rad
	Ry << cos(y) , 0, sin(y),
		  0,       1,   0,
		  -sin(y), 0, cos(y);
	return Ry;
}

Eigen::Matrix3d IMUboardAPI::rotZ(float z)
{
	Eigen::Matrix3d Rz;
	z = z *(M_PI/180); // From deg to Rad
	Rz << cos(z), -sin(z), 0,
		  sin(z), cos(z),  0,
		  0,        0,     1;
	return Rz;
}

Eigen::Matrix3d IMUboardAPI::rot(float x, float y, float z)
{
	Eigen::Matrix3d Rx, Ry, Rz, Rf;
	
	Rx = rotX(x);
	Ry = rotY(y);
	Rz = rotZ(z);

	Rf = Rz*Ry*Rx;
	return Rf;
}
//Skew Matrix
Eigen::Matrix3d IMUboardAPI::skew(Eigen::Vector3d a)
{
		Eigen::Matrix3d q;
		 q(0,0) =  0;          q(0,1) = -a(2);       q(0,2) = a(1);
		 q(1,0) =  a(2);       q(1,1) =  0;          q(1,2) = -a(0);
		 q(2,0) = -a(1);       q(2,1) =  a(0);       q(2,2) = 0;
			return q;
}
//From Quaternion to Rotation Matrix
Eigen::Matrix3d IMUboardAPI::Quat2Rot(Eigen::Vector4d Q_in)
{
	Eigen::Matrix3d RQ;
	float q0, q1, q2, q3;
	q0 = Q_in(0);
	q1 = Q_in(1);
	q2 = Q_in(2);
	q3 = Q_in(3);
	
	RQ <<         q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2),
				  2*(q1*q2 + q0*q3), q0*q0 + q2*q2 - q1*q1 - q3*q3, 2*(q2*q3 - q0*q1),
				  2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0*q0 + q3*q3 - q1*q1 - q2*q2;
	return RQ;
}

Eigen::Vector4d IMUboardAPI::ConjQ(Eigen::Vector4d Q_in) 
{
	Eigen::Vector4d Q_out;
	Q_out(0) =  Q_in(0);
	Q_out(1) = -Q_in(1);
	Q_out(2) = -Q_in(2);
	Q_out(3) = -Q_in(3);
	return Q_out;
}

Eigen::Vector4d IMUboardAPI::QxQ(Eigen::Vector4d Q_1, Eigen::Vector4d Q_2) 
{
    Eigen::Vector4d Q_out;
		Q_out(0) = Q_1(0)*Q_2(0) - (Q_1(1)*Q_2(1) + Q_1(2)*Q_2(2) + Q_1(3)*Q_2(3));
		Q_out(1) = Q_1(0)*Q_2(1) + Q_1(1)*Q_2(0) + (Q_1(2)*Q_2(3) - Q_1(3)*Q_2(2));
		Q_out(2) = Q_1(0)*Q_2(2) + Q_1(2)*Q_2(0) + (Q_1(3)*Q_2(1) - Q_1(1)*Q_2(3));
		Q_out(3) = Q_1(0)*Q_2(3) + Q_1(3)*Q_2(0) + (Q_1(1)*Q_2(2) - Q_1(2)*Q_2(1));
	return Q_out;
}

Eigen::Vector4d IMUboardAPI::Rot2Quat(Eigen::Matrix3d R_in)
{
	Eigen::Vector4d q_out;
	q_out (0) = 0.5 * sqrt(R_in(0,0) +R_in(1,1) +R_in(2,2) +1 );
	q_out (1) = 0.5 * sgn(R_in(2,1)-R_in(1,2)) * sqrt(R_in(0,0)-R_in(1,1) - R_in(2,2) + 1);
	q_out (2) = 0.5 * sgn(R_in(0,2)-R_in(2,0)) * sqrt(-R_in(0,0)+R_in(1,1) - R_in(2,2) + 1);
	q_out (3) = 0.5 * sgn(R_in(1,0)-R_in(0,1)) * sqrt(-R_in(0,0)-R_in(1,1) + R_in(2,2) + 1);

	return q_out; 
}

Eigen::Vector3d IMUboardAPI::Quat2Angle(Eigen::Vector4d Q_in)
{
	Eigen::Vector3d Angles_out;
 		Angles_out(0) = atan2(2*Q_in(1)*Q_in(2) - 2*Q_in(0)*Q_in(3), 2*Q_in(0)*Q_in(0) + 2*Q_in(1)*Q_in(1)-1); //YAW
	 	Angles_out(1) = -asin(2*Q_in(1)*Q_in(3) + 2*Q_in(0)*Q_in(2)); //PITCH
 	 	Angles_out(2) = atan2(2*Q_in(2)*Q_in(3) - 2*Q_in(0)*Q_in(1), 2*Q_in(0)*Q_in(0) + 2*Q_in(3)*Q_in(3)-1); // ROLL
	return Angles_out; 	 	
}

Eigen::Vector3d IMUboardAPI::Rot2Angle(Eigen::Matrix3d R_in)
{
	Eigen::Vector3d Angles_out;
 		Angles_out(0) = atan2(R_in(1,0),R_in(0,0)); //YAW
	 	Angles_out(1) = atan2(-R_in(2,0), sqrt(R_in(2,1)*R_in(2,1) + R_in(2,2)*R_in(2,2))); //PITCH
 	 	Angles_out(2) = atan2(R_in(2,1),R_in(2,2)); // ROLL
	return Angles_out; 	 	
}

