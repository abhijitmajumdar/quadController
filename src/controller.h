#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <RTIMULib.h>

//#define qPLUSCONFIG 1

/*
	 TOP VIEW
	cwm1 \ x / m4ccw
	       \|/
	     y--z
	       / \
	ccwm2 /   \ m3cw
*/

//Kd should be -ve
struct qPIDvariables {
	float Kp;
	float Ki;
	float Kd;
	float integratedSum;
	float previousError;
	uint64_t previousTime;
	float targetValue;
	float computedPIDvalue;
	float boundIterm;
	float KpAngular;
};

struct qMotorThrust {
	float m1Value;
	float m2Value;
	float m3Value;
	float m4Value;
	float mMinBound;
	float mMaxBound;
	float mMinPID;
	float mMaxPID;
};

class qControl{
	qPIDvariables* qPIDval[3]; //Roll(x),Pitch(y),Yaw(z)
	qMotorThrust* qM;
	RTIMU_DATA* currentVal;
	int *altSonar;
	float* thrtl;
	public:
		qControl(qPIDvariables*, qPIDvariables*, qPIDvariables*, qMotorThrust*, RTIMU_DATA*, int*, float*);
		void compute();
	private:
		void qPIDcompute(float currentAngleValue, float currentGyroValue, uint64_t currentTime, qPIDvariables* qPIDv);
		float qConstrain(float value, float min, float max);
		void qCalibrateMotor();
};

#endif //_CONTROLLER_H_
