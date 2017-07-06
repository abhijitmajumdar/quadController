#ifndef _CONFIGRATION_H
#define _CONFIGRATION_H

#include <iostream>
#include <fstream> //configFile File handler
#include <sstream> //line String handler
#include <string>
#include <map>

enum{
	ROS_AND_RC = 0,
	ROS_ONLY,
	RC_ONLY
};

/*
 * qConstants: contains a map of constants that can be refrenced using the constant name as strings
 * Example usage: qConstants["TIME_TO_COMPUTE"]
 * Can be used to read or write to the constant value
 * MODE: ROS_AND_RC->0, ROS_ONLY->1, RC_ONLY->2
 * Use 1->True, 0->False for boolean vales
 */
extern std::map<std::string,float> qConstants;

namespace qConfig{
	void read_config_file(std::string configFileName);
	void print_loaded_constants();
}

#endif //_CONFIGRATION_H
