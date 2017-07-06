#include "configuration.h"

std::map<std::string,float> qConstants = {
	{"TIME_TO_COMPUTE",5000},
	{"TIME_TO_UPDATEMOTOR",10000},
	{"TIME_TO_ROS_PUBLISH",200000},
	{"TIME_TO_ROS_SPIN",200000},
	{"TIME_TO_ARM",3000000},
	{"TIME_TO_DEBUG_DISPLAY",1000000},
	{"TIME_TO_GET_RCUSB",20000},
	{"QuadID",4745},
	{"I_THROTTLE_TRIGGER",1.5},
	{"PD_THROTTLE_TRIGGER",1.3},
	{"YAW_PA",2.0},
	{"YAW_P",0},
	{"YAW_I",0},
	{"YAW_D",0},
	{"ROLL_PA",5.0},
	{"ROLL_P",0},
	{"ROLL_I",0},
	{"ROLL_D",0},
	{"PITCH_PA",5.0},
	{"PITCH_P",0},
	{"PITCH_I",0},
	{"PITCH_D",0},
	{"CH_THROTTLE_MIN_CALIBRATE",1.0},
	{"CH_THROTTLE_MAX_CALIBRATE",2.0},
	{"CH_PITCH_CALIBRATE",1.5},
	{"CH_ROLL_CALIBRATE",1.5},
	{"CH_YAW_CALIBRATE",1.5},
	{"CH_GEAR_CALIBRATE",1.5},
	{"CH_PITCH_VARIANCE",0.0},
	{"CH_ROLL_VARIANCE",0.0},
	{"CH_YAW_VARIANCE",0.0},
	{"CH_PITCH_CALIBRATE_ROS",0.0},
	{"CH_ROLL_CALIBRATE_ROS",0.0},
	{"CH_YAW_CALIBRATE_ROS",0.0},
	{"debugDisplay",0},
	{"MODE",0}
};

//Make sure to compile with "-std=c++11"
void qConfig::read_config_file(std::string configFileName)
{
	std::fstream configFile;
	configFile.open(configFileName,std::fstream::out | std::fstream::in);
	if(configFile.is_open())
	{
		//Load the configurations
		std::string line;
		while(std::getline(configFile,line))
		{
			std::istringstream is_line(line);
			std::string key;
			if( getline(is_line, key, '=') )
			{
				std::string value;
				if(getline(is_line, value) )
				{
					if(qConstants.count(key)>0)
						qConstants[key] = stof(value);
					else
						std::cout<<"Could not find "<<key<<"\n";
				}
			}
		}

		//Close file at the end
		configFile.close();
	}
	else
	{
		std::cout<<"Error reading config file. Loading deafult values\n";
	}
}

void qConfig::print_loaded_constants()
{
	for(std::map<std::string,float>::iterator it=qConstants.begin(); it!=qConstants.end(); ++it)
		std::cout << it->first << " => " << it->second << '\n';
}
