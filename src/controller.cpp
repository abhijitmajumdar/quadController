#include "controller.h"

#define M1_OFFSET 0	//(float)0.09
#define M2_OFFSET 0	//(float)0.20
#define M3_OFFSET 0	//(float)0.20
#define M4_OFFSET 0	//(float)0.18

qControl::qControl(qPIDvariables* vPhi, qPIDvariables* vTheta, qPIDvariables* vGamma, qMotorThrust* qMotor, RTIMU_DATA* currentValues, int *altitudeSonar, float* throttle, float* iThrottleTrigger, float* pdThrottleTrigger)
{
	qPIDval[0] = vPhi;
	qPIDval[1] = vTheta;
	qPIDval[2] = vGamma;
	qM = qMotor;
	currentVal = currentValues;
	altSonar = altitudeSonar;
	thrtl = throttle;
	iTT = iThrottleTrigger;
	pdTT = pdThrottleTrigger;
}

void qControl::compute()
{
	qPIDlimit();
	qPIDcompute(currentVal->fusionPose.x(),currentVal->gyro.x(),currentVal->timestamp,qPIDval[0]);
	qPIDcompute(currentVal->fusionPose.y(),currentVal->gyro.y(),currentVal->timestamp,qPIDval[1]);
	qPIDcompute(0,currentVal->gyro.z(),currentVal->timestamp,qPIDval[2]);
	qCalibrateMotor();
}

void qControl::qPIDlimit()
{
	if(*thrtl>*iTT)
	{
		qPIDval[0]->Ki=qPIDval[0]->KiBuffer;
		qPIDval[1]->Ki=qPIDval[1]->KiBuffer;
		qPIDval[2]->Ki=qPIDval[2]->KiBuffer;
	}
	else
	{
		qPIDval[0]->Ki=0;
		qPIDval[1]->Ki=0;
		qPIDval[2]->Ki=0;
	}
	if(*thrtl>*pdTT)
	{
		qPIDval[0]->Kp=qPIDval[0]->KpBuffer;
		qPIDval[1]->Kp=qPIDval[1]->KpBuffer;
		qPIDval[2]->Kp=qPIDval[2]->KpBuffer;
		qPIDval[0]->Kd=qPIDval[0]->KdBuffer;
		qPIDval[1]->Kd=qPIDval[1]->KdBuffer;
		qPIDval[2]->Kd=qPIDval[2]->KdBuffer;
	}
	else
	{
		qPIDval[0]->Kp=0;
		qPIDval[1]->Kp=0;
		qPIDval[2]->Kp=0;
		qPIDval[0]->Kd=0;
		qPIDval[1]->Kd=0;
		qPIDval[2]->Kd=0;
	}
	if(qPIDval[0]->Ki == 0) qPIDval[0]->integratedSum = 0;
	if(qPIDval[1]->Ki == 0) qPIDval[1]->integratedSum = 0;
	if(qPIDval[2]->Ki == 0) qPIDval[2]->integratedSum = 0;
	if(qPIDval[0]->Kd == 0) qPIDval[0]->previousError = 0;
	if(qPIDval[1]->Kd == 0) qPIDval[1]->previousError = 0;
	if(qPIDval[2]->Kd == 0) qPIDval[2]->previousError = 0;
}

void qControl::qPIDcompute(float currentAngleValue, float currentGyroValue, uint64_t currentTime, qPIDvariables* qPIDv)
{
	float error = qPIDv->KpAngular*(qPIDv->targetValue - currentAngleValue);
	error = (error - currentGyroValue);
	uint64_t dTime = currentTime - qPIDv->previousTime;
	float pTerm = (qPIDv->Kp * error);
	qPIDv->integratedSum += (qPIDv->Ki * error * dTime);
	if((qPIDv->integratedSum) < -(qPIDv->boundIterm))
		(qPIDv->integratedSum) = -(qPIDv->boundIterm);
	if((qPIDv->integratedSum) > (qPIDv->boundIterm))
		(qPIDv->integratedSum) = (qPIDv->boundIterm);
	float dTerm = (qPIDv->Kd * (error - qPIDv->previousError))/dTime;
	float computeValue = pTerm + qPIDv->integratedSum + dTerm;
	qPIDv->previousTime = currentTime;
	qPIDv->previousError = error;
	qPIDv->computedPIDvalue = computeValue;
}

float qControl::qConstrain(float value, float min, float max)
{
	if(value < min)
		return min;
	if(value > max)
		return max;
	return value;
}

void qControl::qCalibrateMotor()
{
#ifdef qPLUSCONFIG
	qM->m1Value = (zDotTerm - qPIDval[0]->computedPIDvalue - qPIDval[2]->computedPIDvalue);
	qM->m2Value = (zDotTerm - qPIDval[1]->computedPIDvalue + qPIDval[2]->computedPIDvalue);
	qM->m3Value = (zDotTerm + qPIDval[0]->computedPIDvalue - qPIDval[2]->computedPIDvalue);
	qM->m4Value = (zDotTerm + qPIDval[1]->computedPIDvalue + qPIDval[2]->computedPIDvalue);
#else
	qM->m1Value = (qPIDval[0]->computedPIDvalue + qPIDval[1]->computedPIDvalue - qPIDval[2]->computedPIDvalue);
	qM->m2Value = (qPIDval[0]->computedPIDvalue - qPIDval[1]->computedPIDvalue + qPIDval[2]->computedPIDvalue);
	qM->m3Value = (- qPIDval[0]->computedPIDvalue - qPIDval[1]->computedPIDvalue - qPIDval[2]->computedPIDvalue);
	qM->m4Value = (- qPIDval[0]->computedPIDvalue + qPIDval[1]->computedPIDvalue + qPIDval[2]->computedPIDvalue);
#endif
	qM->m1Value = qConstrain(qM->m1Value, qM->mMinPID, qM->mMaxPID);
	qM->m2Value = qConstrain(qM->m2Value, qM->mMinPID, qM->mMaxPID);
	qM->m3Value = qConstrain(qM->m3Value, qM->mMinPID, qM->mMaxPID);
	qM->m4Value = qConstrain(qM->m4Value, qM->mMinPID, qM->mMaxPID);
	
	qM->m1Value += (*thrtl+M1_OFFSET);
	qM->m2Value += (*thrtl+M2_OFFSET);
	qM->m3Value += (*thrtl+M3_OFFSET);
	qM->m4Value += (*thrtl+M4_OFFSET);
	
	qM->m1Value = qConstrain(qM->m1Value, qM->mMinBound, qM->mMaxBound);
	qM->m2Value = qConstrain(qM->m2Value, qM->mMinBound, qM->mMaxBound);
	qM->m3Value = qConstrain(qM->m3Value, qM->mMinBound, qM->mMaxBound);
	qM->m4Value = qConstrain(qM->m4Value, qM->mMinBound, qM->mMaxBound);
}
