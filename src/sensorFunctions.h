#ifndef _SENSORFUNCTIONS_H
#define _SENSORFUNCTIONS_H

#include <iostream>
#include <signal.h>
#include <RTIMULib.h>
#include <wiringPi.h>
#include <wiringSerial.h>

void IMU_init(void);
void IMU_spin(void);
RTIMU_DATA IMU_data(void);
void SONAR_init(void);
int SONAR_data(void);
void Sensors_init();

#endif //_SENSORFUNCTIONS_H
