#include "reactiveGraspDetection.h"

// =========================================================================================================
//																					 			 Constructor
// =========================================================================================================
ReactiveGraspDetection::ReactiveGraspDetection()
{
	ROS_INFO_STREAM("START GLOVE COMMUNICATION");
	initGloveCommunication();

	ROS_INFO_STREAM("START ACQUISITION");
	initParameters();


	for(int i=0; i<300; i++)
	{
		processData();
	}

	updateLogFiles();

}


// =========================================================================================================
//																					 			 Desctructor
// =========================================================================================================
ReactiveGraspDetection::~ReactiveGraspDetection()
{
	std::cout << "\n\nInterrupt signal received\n";
    glove_.stopPSoC(); // Stop Communication
	std::cout << "\n\n\n\r\033[31m\033[1mSHUTDOWN ReactiveGraspDetection\033[0m \r\n\n\n";

	// closes log files if previously opened
	if (log_file_accel_raw_.is_open()) 
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_raw_ + ".dat\n";
		log_file_accel_raw_.close();
	}
	if (log_file_accel_filt_.is_open()) 
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_filt_ + ".dat\n";
		log_file_accel_filt_.close();
	}
	if (log_file_accel_map_.is_open()) 
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_map_ + ".dat\n";
		log_file_accel_map_.close();
	}
	if (log_file_gyro_raw_.is_open()) 
	{
		std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_gyro_ + ".dat\n";
		log_file_gyro_raw_.close();
	}
}




/**********************************************************************************************************/
/*																								  		  */
/*																								  		  */
/*												FUNCTIONS		                                          */
/*																								    	  */
/*																								  		  */
/**********************************************************************************************************/
void ReactiveGraspDetection::appendToLogFile(std::ofstream *log_file, const std::vector<Sensor> data) 
{
  // stores all data sensor in the given log file 
  *log_file << std::fixed ;
  for (int i=0; i< (int) data.size(); i++) 
  {
    *log_file << ";" << data[i].x // data.x.back()
              << ";" << data[i].y // data.y.back()
              << ";" << data[i].z; // data.z.back();
  }
  *log_file << std::endl;
}

// =========================================================================================================
//																							     Filter Data
// =========================================================================================================
void ReactiveGraspDetection::filterData()
{
	float f_cut_ = 5;
	// float dt_ = 0.012;
	std::cout << "time: "  << dt_ << std::endl;

	float RC = 1/(2*M_PI*f_cut_);
	float alpha = RC / (RC+dt_);

	IMUData tmp;
	tmp.acc.resize(nIMU_);
	tmp.gyro.resize(nIMU_);



	// first acquisition
	if(filtered_data_.size()<3)
	{
		for (int i=0; i<nIMU_; i++)
		{
			tmp.acc[i].x =  raw_data_[0].acc[i].x;
			tmp.acc[i].y =  raw_data_[0].acc[i].y;
			tmp.acc[i].z =  raw_data_[0].acc[i].z;

			tmp.gyro[i].x =  raw_data_[0].gyro[i].x;
			tmp.gyro[i].y =  raw_data_[0].gyro[i].y;
			tmp.gyro[i].z =  raw_data_[0].gyro[i].z;
		}

		filtered_data_.push_back(tmp);
	}


	// update the data windows 
	if( (int) filtered_data_.size() == samples_)
		filtered_data_.erase(filtered_data_.begin());
	
		
	int end_filt = filtered_data_.size() - 1;
	int end_raw  = raw_data_.size() - 1;

	// filter the data (with two raw_data_ at least)
	if(end_raw > 2)
	{	
		for (int j=0; j<nIMU_; j++)
		{
			tmp.acc[j].x =  alpha * (filtered_data_[end_filt].acc[j].x + raw_data_[end_raw].acc[j].x - raw_data_[end_raw-1].acc[j].x);
			tmp.acc[j].y =  alpha * (filtered_data_[end_filt].acc[j].y + raw_data_[end_raw].acc[j].y - raw_data_[end_raw-1].acc[j].y);
			tmp.acc[j].z =  alpha * (filtered_data_[end_filt].acc[j].z + raw_data_[end_raw].acc[j].z - raw_data_[end_raw-1].acc[j].z);
		}

		filtered_data_.push_back(tmp);
	}


	// record all data if required
	if(_RECORD_ALL_DATA_)
	{
		appendToLogFile(&log_file_ACC_filt_, tmp.acc);
	}


}


// =========================================================================================================
//																					Init Glove Communication
// =========================================================================================================
void ReactiveGraspDetection::initGloveCommunication()
{
	/************************
	*	    IMUboardAPI		*
	************************/
	glove_.p_.port 		= (char*)"/dev/ttyUSB0";
	glove_.p_.baudRate 	= 1000000;
	glove_.p_.nIMU		= 17; 															 // XXX DA MODIFICARE PER IL GUANTO PER IL REACTIVE GRASP
	glove_.p_.byteIMU	= 14; // 14 for acc - gyro; 22 for acc - gyro - mag

	nIMU_ = glove_.p_.nIMU;
	glove_.initPSoC();	
}


// =========================================================================================================  
//																							  Init Variables
// =========================================================================================================
void ReactiveGraspDetection::initParameters()
{ 
	//load parameters form ros server
	node_handle_.param("samples", samples_, 60);


 	// stores in 'date_time_' the current time converted into a handful form (date/time format YYYYMMDD_HHMMSS)
	std::time_t raw_time;
	char buffer[16];
	std::time(&raw_time);
	std::strftime(buffer, 16, "%G%m%d_%H%M%S", std::localtime(&raw_time));
	date_time_ = buffer;

 	log_file_base_path_ = "/home/emanuele/catkin_ws/src/reactive_grasp/logs/";
	log_file_name_raw_  = "accelerations_raw";
	log_file_name_filt_ = "accelerations_filtered";
	log_file_name_map_  = "accelerations_map";
	log_file_name_gyro_ = "gyro_velocities_raw";
	log_file_name_ACC_raw_ = "ACC_raw";
	log_file_name_ACC_filt_ = "ACC_filt";

 	// creates folder if it doesn't exist
	std::string command = "mkdir -p " + log_file_base_path_;
	system(command.c_str());

	// creates folder if it doesn't exist
	std::string local_folder;
	local_folder = log_file_base_path_ + date_time_;
	command = "mkdir -p " + local_folder;
	system(command.c_str());
	// opens log files
	// log_file_accel_raw_.open(local_folder + "/" + date_time_ + "_" + log_file_name_raw_ + ".dat");
	// log_file_accel_filt_.open(local_folder + "/" + date_time_ + "_" + log_file_name_filt_ + ".dat");
	// log_file_accel_map_.open(local_folder + "/" + date_time_ + "_" + log_file_name_map_ + ".dat");
	// log_file_gyro_raw_.open(local_folder + "/" + date_time_ + "_" + log_file_name_gyro_ + ".dat");
	log_file_accel_raw_.open(local_folder + "/" +  log_file_name_raw_ + ".dat");
	log_file_accel_filt_.open(local_folder + "/" +  log_file_name_filt_ + ".dat");
	log_file_accel_map_.open(local_folder + "/" +  log_file_name_map_ + ".dat");
	log_file_gyro_raw_.open(local_folder + "/" +  log_file_name_gyro_ + ".dat");
	log_file_ACC_raw_.open(local_folder + "/" + log_file_name_ACC_raw_ + ".dat");
	log_file_ACC_filt_.open(local_folder + "/" + log_file_name_ACC_filt_ + ".dat");

	std::cout<< log_file_base_path_ + date_time_ + "_" + log_file_name_raw_ + ".dat"<< std::endl;
	// statistics variables initialization
	start_time_ = ros::Time::now();
}



// =========================================================================================================
//																					 	   		Process Data
// =========================================================================================================
// void ReactiveGraspDetection::processData(ros::Duration acquisition_time)
void ReactiveGraspDetection::processData()
{
	updateRawData();
	filterData();
}



// =========================================================================================================
//																					 	   		 Update Data
// =========================================================================================================
void ReactiveGraspDetection::updateRawData()
{	

	// erase oldest sample
	if( (int) raw_data_.size()==samples_)
		raw_data_.erase(raw_data_.begin());
	
	IMUData tmp;
	tmp.acc.resize(nIMU_);
	tmp.gyro.resize(nIMU_);
	// read glove and record the time between two evetns
	ros::Time start, stop;
	start = ros::Time::now();
	
	glove_.readPSoC();

	stop = ros::Time::now();
	dt_  = stop.toSec() - start.toSec();

	for(int j=0; j<nIMU_; j++)
	{
		tmp.acc[j].x = glove_.Acc_(j,0);
		tmp.acc[j].y = glove_.Acc_(j,1);
		tmp.acc[j].z = glove_.Acc_(j,2);

		tmp.gyro[j].x = glove_.Gyro_(j,0);
		tmp.gyro[j].y = glove_.Gyro_(j,1);
		tmp.gyro[j].z = glove_.Gyro_(j,2);
	}

	// update raw_data
	raw_data_.push_back(tmp);

	// record all data if required
	if(_RECORD_ALL_DATA_)
	{
		appendToLogFile(&log_file_ACC_raw_, tmp.acc);
	}

}


// =========================================================================================================
//																					 	    Update Log Files
// =========================================================================================================
void ReactiveGraspDetection::updateLogFiles()
{	
  	for(int i=0; i< (int) raw_data_.size(); i++)
  	{	
  		appendToLogFile(&log_file_accel_raw_, raw_data_[i].acc);
  		appendToLogFile(&log_file_accel_filt_, filtered_data_[i].acc);
  		appendToLogFile(&log_file_gyro_raw_, raw_data_[i].gyro);
  	}
}


