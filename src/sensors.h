#ifndef _SENSORS_H
#define _SENSORS_H

#include <iostream>
#include <signal.h>
#include <RTIMULib.h>
#include <wiringPi.h>
#include <wiringSerial.h>

void IMU_init(std::string *package_path);
void IMU_spin(void);
RTIMU_DATA IMU_data(void);
void SONAR_init(void);
int SONAR_data(void);
void Sensors_init(std::string *package_path);

#endif //_SENSORS_H
