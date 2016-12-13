#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include "actuatorFunctions.h"
#include "sensorFunctions.h"
#include "controller.h"
#include <quadMsgs/qParameters.h>
#include <quadMsgs/qStatus.h>

#define TIME_TO_COMPUTE 20000 //uS
#define TIME_TO_UPDATEMOTOR 20000 //uS
#define TIME_TO_ROS_PUBLISH 500000 //uS
#define TIME_TO_ROS_SPIN 200000 //uS
#define TIME_TO_ARM 3000000 //uS
#define TIME_TO_DEBUG_DISPLAY 1000000 //uS

#define QuadID 0x1289

using namespace std;

static qPIDvariables vPhi,vTheta,vGamma;
static qMotorThrust vMotor;
static RTIMU_DATA imud;
static int groundDistance;
static bool debugDisplay = false;
static float throttle = 1.0;
static bool Arm = false, Armed = false;

void gotquadPosition(const std_msgs::String::ConstPtr& msg)
{
	;
}

void gotquadArm(const std_msgs::Bool::ConstPtr& msg)
{
	bool qArm = msg->data;
	if(qArm == true)
	{
		Arm = true;
	}
	else
	{
		Arm = false;
		Armed = false;
	}
}

void gotquadParam(const quadMsgs::qParameters::ConstPtr& msg)
{
	if(msg->qID == QuadID)
	{
		int32_t qSpeed = msg->qThrottle;
		if((qSpeed>=0) & (qSpeed<=120))
			throttle=(((float)qSpeed)/100)+(1.0);
		else
			throttle = 1.0;
		int32_t qP = msg->qP;
		if((qP>=0) & (qP<10000))
		{
			vPhi.Kp=((float)qP)/1000;
			vTheta.Kp=((float)qP)/1000;
		}
		int32_t qI = msg->qI;
		if((qI>=0) & (qI<1000))
		{
			vPhi.Ki=((float)qI)/1000000;
			vTheta.Ki=((float)qI)/1000000;
		}
		if(qI==0)
		{
			vPhi.integratedSum = 0;
			vTheta.integratedSum = 0;
		}
		int32_t qD = msg->qD;
		if((qD>=0) & (qD<100))
		{
			vPhi.Kd=((float)qD)*1000;
			vTheta.Kd=((float)qD)*1000;
		}
		if(qD==0)
		{
			vPhi.previousError = 0;
			vTheta.previousError = 0;
		}
		cout<<"T,P,I,D = "<<throttle<<','<<vPhi.Kp<<','<<vPhi.Ki<<','<<vPhi.Kd<<"\n";
	}
	else{
		cout<<"Message not for me: "<<msg->qID;
	}
}

void initPIDvalues(void)
{
	vPhi.Kp = 0;
	vPhi.Ki = 0;
	vPhi.Kd = 0;
	vPhi.integratedSum = 0;
	vPhi.previousError = 0;
	vPhi.previousTime = 0;
	vPhi.targetValue = 0;
	vPhi.boundIterm = 0.35;
	
	vTheta.Kp = 0;
	vTheta.Ki = 0;
	vTheta.Kd = 0;
	vTheta.integratedSum = 0;
	vTheta.previousError = 0;
	vTheta.previousTime = 0;
	vTheta.targetValue = 0;
	vTheta.boundIterm = 0.35;
	
	vGamma.Kp = 0.1;
	vGamma.Ki = 0;
	vGamma.Kd = 0;
	vGamma.integratedSum = 0;
	vGamma.previousError = 0;
	vGamma.previousTime = 0;
	vGamma.targetValue = 0;
	vGamma.boundIterm = 0.35;
	
	vMotor.m1Value = 0;
	vMotor.m2Value = 0;
	vMotor.m3Value = 0;
	vMotor.m4Value = 0;
	vMotor.mMinBound = 1.0;
	vMotor.mMaxBound = 2.2;
	vMotor.mMinPID = -0.5;
	vMotor.mMaxPID = 0.5;
}

int main(int argc, char **argv)
{
	uint64_t timeToCompute = 0,timeToRosPublish = 0, timeToUpdateMotor = 0;
	uint64_t timeToRosSpin = 0, timeNow = 0;
	uint64_t timeSinceArm = 0;
	uint64_t timeToDebugDisplay=0;
	Sensors_init();
	Actuators_init();
	initPIDvalues();
	qControl quadController(&vPhi, &vTheta, &vGamma, &vMotor, &imud, &groundDistance, &throttle);
	
	ros::init(argc, argv, "quadController");
	ros::NodeHandle n;
	ros::Publisher quadStatus = n.advertise<quadMsgs::qStatus>("quadStatus", 10);
	ros::Subscriber quadPosition = n.subscribe("quadPosition", 10, gotquadPosition);
	ros::Subscriber quadArm = n.subscribe("quadArm", 10, gotquadArm);
	ros::Subscriber quadParam = n.subscribe("quadParam", 10, gotquadParam);
	//ros::Rate loop_rate(1000);

	LED_led(AMBER,HIGH);
	setMotor(1.0, 1.0, 1.0, 1.0);
	PWM_engage(HIGH);
	LED_led(BLUE,HIGH);

	while(ros::ok())
	{
		IMU_spin();
		timeNow = RTMath::currentUSecsSinceEpoch();
		if(timeNow-timeToCompute>TIME_TO_COMPUTE)
		{
			timeToCompute = timeNow;
			imud = IMU_data();
			imud.fusionPose.setZ(0);
			imud.fusionPose.setY(0);
			//groundDistance = SONAR_data();
			quadController.compute();
		}
		
		if(timeNow-timeToUpdateMotor>TIME_TO_UPDATEMOTOR)
		{
			timeToUpdateMotor = timeNow;
			if(Armed == true)
			{
				//setMotor(1.0, 1.0, 1.0, 1.0);
				setMotor(vMotor.m1Value, vMotor.m2Value, vMotor.m3Value, vMotor.m4Value);
				//setMotor(1.0, vMotor.m2Value, 1.0, vMotor.m4Value);
			}
			else
			{
				setMotor(1.0, 1.0, 1.0, 1.0);
			}
			if(Arm == true)
			{
				timeSinceArm = timeNow;
				Arm = false;
				Armed = true;
			}
			if(((timeNow-timeSinceArm)>TIME_TO_ARM)&(Armed==true))
			{
				Armed = false;
				break;
			}
		}
		
		if(timeNow-timeToRosSpin>TIME_TO_ROS_SPIN)
		{
			timeToRosSpin = timeNow;
			ros::spinOnce();
		}

		if(timeNow-timeToRosPublish>TIME_TO_ROS_PUBLISH)
		{
			timeToRosPublish = timeNow;
			quadMsgs::qStatus msg;
			msg.qID = QuadID;
			msg.qM1 = vMotor.m1Value;
			msg.qM2 = vMotor.m2Value;
			msg.qM3 = vMotor.m3Value;
			msg.qM4 = vMotor.m4Value;
			msg.qX = imud.fusionPose.x();
			msg.qY = imud.fusionPose.y();
			msg.qZ = imud.fusionPose.z();
			quadStatus.publish(msg);
		}
		
		if(debugDisplay == true)
			if(timeNow-timeToDebugDisplay>TIME_TO_DEBUG_DISPLAY)
			{
				timeToDebugDisplay = timeNow;
				cout << vMotor.m1Value << "\t";
				cout << vMotor.m2Value << "\t";
				cout << vMotor.m3Value << "\t";
				cout << vMotor.m4Value << "\t";
				cout <<"\n";
				cout << imud.timestamp << "\n";
			}
	}
	LED_led(AMBER,LOW);
	setMotor(1.0, 1.0, 1.0, 1.0);
	//PWM_engage(LOW);
	LED_led(BLUE,LOW);
	return 0;
}
