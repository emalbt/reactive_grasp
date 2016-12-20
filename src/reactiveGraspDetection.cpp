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

	parseAccelerationMap();

	for(;;)
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
    // glove_.stopPSoC(); // Stop Communication
	std::cout << "\n\n\n\r\033[31m\033[1mSHUTDOWN ReactiveGraspDetection\033[0m \r\n\n\n";

	// closes log files if previously opened
	if (log_file_accel_raw_.is_open()) 
	{
		std::cout << "  + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_raw_ + ".dat\n";
		log_file_accel_raw_.close();
	}
	if (log_file_accel_filt_.is_open()) 
	{
		std::cout << "  + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_filt_ + ".dat\n";
		log_file_accel_filt_.close();
	}
	if (log_file_accel_map_.is_open()) 
	{
		std::cout << "  + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_map_ + ".dat\n";
		log_file_accel_map_.close();
	}
	if (log_file_gyro_raw_.is_open()) 
	{
		std::cout << "  + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_gyro_ + ".dat\n";
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

// =========================================================================================================
//																					 	  Append to Log File
// =========================================================================================================
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
//																					 	   	  Detect Contact
// =========================================================================================================
int ReactiveGraspDetection::detectContact()
{   
	// finds which IMU has the biggest acceleration from all the axes (at 'sc' sample)
	std::vector<double> accel_abs_values;

	//sample check
	int sc = check_sample_;
	
	for(int i=0; i<nIMU_; i++)
	{
		accel_abs_values.push_back( filtered_data_[sc].abs_contribution[i] );
	}

	std::vector<double>::const_iterator it_accel_max = std::max_element(accel_abs_values.begin(), accel_abs_values.end());
	// evaluates if the max(abs(xyz)) is greater than a specific threshold
  	if (*it_accel_max > contact_threshold_) 
  	{
    	int imu_id = it_accel_max - accel_abs_values.begin();
    	// checks for outliers (following 2 samples must be relevant and at least one opposite in sign to the first)
	    if( filtered_data_[sc+1].abs_contribution[imu_id] > contact_threshold_ 
	    	&& filtered_data_[sc+2].abs_contribution[imu_id] > contact_threshold_ )
	    {
	    	// TODO add hand_closed_ flag

	    	ROS_DEBUG_STREAM("[Detection::detectContact] IMU: " << imu_id );
	    	return imu_id;
		}
    	
  	}

  	return -1;
}

// =========================================================================================================
//																					 Extract Grasp Primitive
// =========================================================================================================
std::string ReactiveGraspDetection::extractGraspPrimitive(std::vector<double> data, int imu_id) 
{
	double xcorr_value;
	std::map<std::string, double> xcorr_values_map;
	bool local_flag = false;

	for (auto const &pair : accel_map_) 
	{
		if(pair.second.at("id").front() == imu_id)
		{	
	    	xcorr_value = xcorr(data, pair.second.at("samples"));
	    	xcorr_values_map.insert(std::make_pair(pair.first, std::abs(xcorr_value)));
	    	local_flag = true;
		}	
	}
	auto it_max = std::max_element(xcorr_values_map.begin(), xcorr_values_map.end(),
                                 [](const std::map<std::string, double>::value_type x,
                                    const std::map<std::string, double>::value_type y){ return x.second < y.second; });
  	// mia verifica per monitorare il valore della soglia xcorr_threshold_
  	if(local_flag)
    	ROS_INFO_STREAM("[Detection::extractGraspPrimitive] xcorr_value: " << it_max->second );
  	
  	if (it_max->second >= xcorr_threshold_)
		return it_max->first;
	else
		return ("false");
}

// =========================================================================================================
//																							     Filter Data
// =========================================================================================================
void ReactiveGraspDetection::filterData()
{
	float f_cut_ = 5;
	// float dt_ = 0.012;
	// std::cout << "time: "  << dt_ << std::endl;

	float RC = 1/(2*M_PI*f_cut_);
	float alpha = RC / (RC+dt_);

	IMUData tmp;
	tmp.acc.resize(nIMU_);
	tmp.gyro.resize(nIMU_);
	tmp.abs_contribution.resize(nIMU_);


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
			
			tmp.abs_contribution[i] = std::abs(tmp.acc[i].x) + std::abs(tmp.acc[i].y) + std::abs(tmp.acc[i].z); 
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
			
			tmp.abs_contribution[j] = std::abs(tmp.acc[j].x) + std::abs(tmp.acc[j].y) + std::abs(tmp.acc[j].z); 
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
//																							  Init Variables
// =========================================================================================================
void ReactiveGraspDetection::initParameters()
{ 
	//load parameters form ros server
	node_handle_.param("samples", samples_, 45);
	node_handle_.param("contact_threshold", contact_threshold_, 0.3);
	node_handle_.param("xcorr_threshold", xcorr_threshold_, 0.7);
	node_handle_.param("check_sample", check_sample_, 5);



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
//																					  Parse Acceleration Map
// =========================================================================================================
void ReactiveGraspDetection::parseAccelerationMap() 
{
  ROS_INFO_STREAM("[Detection::parseAccelerationMap] Parsing object parameters from YAML configuration file...");

  std::string base_name = "approaching_directions";
  std::vector<std::string> approaching_directions;
  XmlRpc::XmlRpcValue list;
  if (!node_handle_.getParam("/" + base_name, list)) 
  {
    ROS_ERROR_STREAM("[Detection::parseAccelerationMap] Can't find '" + base_name + "' in YAML configuration file.");
    return;
  }
  for (auto it = list.begin(); it != list.end(); it++) 
  {
    approaching_directions.push_back(it->first);
  }

  for (auto const &direction : approaching_directions) 
  {
    std::string param_name;
    std::string field_name;
    std::vector<double> samples;
    std::vector<double> position;
    std::vector<double> orientation;
    std::vector<double> id;
    std::map<std::string, std::vector<double>> map;

    field_name = "samples";
    param_name = "/"  + base_name + "/"  + direction + "/"  + field_name;
    if (!node_handle_.getParam(param_name, samples)) 
    {
      ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
      continue;
    }
    map.insert(std::make_pair(field_name, samples));

    field_name = "position";
    param_name = "/"  + base_name + "/" + direction + "/"  + field_name;
    if (!node_handle_.getParam(param_name, position)) 
    {
      ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
      continue;
    }
    map.insert(std::make_pair(field_name, position));

    field_name = "orientation";
    param_name = "/"  + base_name + "/"  + direction + "/"  + field_name;
    if (!node_handle_.getParam(param_name, orientation)) 
    {
      ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
      continue;
    }
    map.insert(std::make_pair(field_name, orientation));

    field_name = "id";
    param_name = "/"  + base_name + "/"  + direction + "/"  + field_name;
    if (!node_handle_.getParam(param_name, id)) 
    {
      ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
      continue;
    }
    map.insert(std::make_pair(field_name, id));

    accel_map_.insert(std::make_pair(direction, map));
  }
}


// =========================================================================================================
//																					 	   		Process Data
// =========================================================================================================
// void ReactiveGraspDetection::processData(ros::Duration acquisition_time)
void ReactiveGraspDetection::processData()
{
	updateRawData();
	filterData();

	if( (int) filtered_data_.size()>=samples_)
	{
		int imu_id = detectContact();
		if(imu_id >= 0)
		{
			ROS_INFO_STREAM("[Detection::processData] Contact Detected on IMU " << imu_id);
			std::vector<double> acc_imuID_to_compare =  toVector(imu_id);
			std::string approaching_direction = extractGraspPrimitive(acc_imuID_to_compare, imu_id);
			if (approaching_direction != "false")
			{
				std::cout<<"questa è la risp: " << approaching_direction.c_str() << std::endl;
				getchar();
			}
	      	// ROS_INFO_STREAM("[Detection::processData] Cross-Correlation Detection on '" << approaching_direction << "'");
		}
	}
}


// =========================================================================================================
//																					 	   		   to Vector
// =========================================================================================================
std::vector<double> ReactiveGraspDetection::toVector( int imu_id)
{	
	std::vector<double> data_out;
	for(int i=0; i<samples_; i++)												// XXX PER ORA COFRONTO SOLO UNA IMU, EVENTUALMENTE DA CAMBIARE
	{
		data_out.push_back(filtered_data_[i].acc[imu_id].x);
		data_out.push_back(filtered_data_[i].acc[imu_id].y);
		data_out.push_back(filtered_data_[i].acc[imu_id].z);
	}

	return data_out;
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

// =========================================================================================================
//																					 	    		  X Corr
// =========================================================================================================
double ReactiveGraspDetection::xcorr(std::vector<double> x, std::vector<double> y)
{

	double mean_x = std::accumulate(x.begin(), x.end(), 0.0) / samples_;
  	double mean_y = std::accumulate(y.begin(), y.end(), 0.0) / samples_;

	// numerator compute
  	double numerator = 0;
  	for (int i=0; i<samples_; i++)
  	{
  		numerator += ((x[i] - mean_x) * (y[i] - mean_y));
  	}

	// denominator compute
	double sx = 0;
	double sy = 0;
	for (int i=0; i<samples_; i++) 
	{
		sx += (x[i] - mean_x) * (x[i] - mean_x);
		sy += (y[i] - mean_y) * (y[i] - mean_y);
	}
	double denominator = std::sqrt(sx*sy);

	return (numerator/denominator);
}




/***********************************************************************************************************/
/*                                                                                                         */
/*                                                                                                         */
/*                                            Glove Functions                                              */
/*                                                                                                         */
/*                                                                                                         */
/***********************************************************************************************************/
// =========================================================================================================
//																					 	   		 Update Data
// =========================================================================================================
void ReactiveGraspDetection::initGloveCommunication()
{
	sub_acc_ = node_handle_.subscribe("/qb_class_imu/acc", 100, &ReactiveGraspDetection::callbackAcc, this);
	sub_gyro_ = node_handle_.subscribe("/qb_class_imu/gyro", 100, &ReactiveGraspDetection::callbackGyro, this);

	acc_flag_ = false;
	gyro_flag_ = false;

	nIMU_ = 7;
	Acc_.resize(nIMU_, 3);
	Gyro_.resize(nIMU_, 3);
	Acc_old_.resize(nIMU_, 3);
	Gyro_old_.resize(nIMU_, 3);

	Acc_.setZero();
	Acc_old_.setZero();
	Gyro_.setZero();
	Gyro_old_.setZero();

	waitGlove();
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
	tmp.abs_contribution.resize(nIMU_);


	// read glove and record the time between two evetns
	ros::Time start, stop;
	start = ros::Time::now();
	
	waitGlove();

	stop = ros::Time::now();
	dt_  = stop.toSec() - start.toSec();

	for(int j=0; j<nIMU_; j++)
	{
		tmp.acc[j].x = Acc_(j,0);
		tmp.acc[j].y = Acc_(j,1);
		tmp.acc[j].z = Acc_(j,2);

		tmp.gyro[j].x = Gyro_(j,0);
		tmp.gyro[j].y = Gyro_(j,1);
		tmp.gyro[j].z = Gyro_(j,2);

		tmp.abs_contribution[j] = std::abs(tmp.acc[j].x) + std::abs(tmp.acc[j].y) + std::abs(tmp.acc[j].z); 
	}

	// update raw_data
	raw_data_.push_back(tmp);

	// record all data if required
	if(_RECORD_ALL_DATA_)
	{
		appendToLogFile(&log_file_ACC_raw_, tmp.acc);
	}

}


// =============================================================================================
//                                                                                     waitGlove
// =============================================================================================
void ReactiveGraspDetection::waitGlove()
{	
	ros::Rate r(2000);
	// wait acc and gyro flag 
	do 
	{
		ros::spinOnce();
		r.sleep();
	}
	while(!acc_flag_ && !gyro_flag_);
	
	acc_flag_ = gyro_flag_ = false;
}



// =============================================================================================
//                                                                                   callbackAcc
// =============================================================================================
void ReactiveGraspDetection::callbackAcc(qb_interface::inertialSensorArray imu)
{
	for(int i=0; i< (int) imu.m.size(); i++)
	{
		Acc_(i,0) = imu.m[i].x;
		Acc_(i,1) = imu.m[i].y;
		Acc_(i,2) = imu.m[i].z;

		// std::cout << i  <<" acc " << Acc_(i,0) << "\t" << Acc_(i,1) << "\t" << Acc_(i,2) << std::endl;
		// if(i==(int) imu.m.size()-1)
		// 	std::cout << "\n";
	}
	 
	 if ((Acc_old_ - Acc_).sum() != 0)
	 	acc_flag_ = true;

	Acc_old_ = Acc_;
}



// =============================================================================================
//                                                                                  callbackGyro
// =============================================================================================
void ReactiveGraspDetection::callbackGyro(qb_interface::inertialSensorArray imu)
{
	for(int i=0; i<(int) imu.m.size(); i++)
	{
		Gyro_(i,0) = imu.m[i].x;
		Gyro_(i,1) = imu.m[i].y;
		Gyro_(i,2) = imu.m[i].z;
		// std::cout << i  <<" gyro " << Gyro_(i,0) << "\t" << Gyro_(i,1) << "\t" << Gyro_(i,2) << std::endl;
		// if(i==(int) imu.m.size()-1)
		// 	std::cout << "\n";
	}

	if ((Gyro_old_ - Gyro_).sum() != 0)
	 	gyro_flag_ = true;

	Gyro_old_ = Gyro_;
}	
