#ifndef _SENSORFUNCTIONS_H
#define _SENSORFUNCTIONS_H

#include <iostream>
#include <signal.h>
#include <RTIMULib.h>
#include <wiringPi.h>
#include <wiringSerial.h>

// On the TGY-iA6 receiver
#define CH_ROLL 0 // CH1: Left-Right/Roll
#define CH_PITCH 1 // CH2: Up-Down/Pitch
#define CH_THROTTLE 2 // CH3: Throttle
#define CH_YAW 3 // CH4: Yaw
#define CH_SWC 4 // CH5: SWC
#define CH_SWB 5 // CH6: SWB

#define PPM_PIN 7
#define nChannels 6

void IMU_init(void);
void IMU_spin(void);
RTIMU_DATA IMU_data(void);
void SONAR_init(void);
int SONAR_data(void);
void pinChange(void);
void RC_PPM_Init(void);
void getRCData(int *vals);
void Sensors_init();

#endif //_SENSORFUNCTIONS_H
