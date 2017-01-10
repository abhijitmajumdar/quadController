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
#include "configLoader.h"

using namespace std;

#define TIME_TO_UPDATE_TARGETANGLE 0
#define TIME_TO_COMPUTE 1
#define TIME_TO_UPDATEMOTOR 2
#define TIME_TO_ROS_PUBLISH 3
#define TIME_TO_ROS_SPIN 4
#define TIME_TO_ARM 5
#define TIME_TO_DEBUG_DISPLAY 6
#define QuadID 0
#define M 0
#define N 1
#define debugDisplay 0

lConst timeConstants[7] = {
	{"TIME_TO_UPDATE_TARGETANGLE",80000},
	{"TIME_TO_COMPUTE",20000},
	{"TIME_TO_UPDATEMOTOR",20000},
	{"TIME_TO_ROS_PUBLISH",200000},
	{"TIME_TO_ROS_SPIN",200000},
	{"TIME_TO_ARM",3000000},
	{"TIME_TO_DEBUG_DISPLAY",1000000}
	};
	
lConst idConstants[1] = {
	{"QuadID",4745}
	};
	
fConst filterConstants[2] = {
	{"M",0.9},
	{"N",0.1}
	};
	
bConst boolConstants[1] = {
	{"debugDisplay",false}
	};

static qPIDvariables vPhi,vTheta,vGamma;
static qMotorThrust vMotor;
static RTIMU_DATA imud;
static int groundDistance;
static float throttle = 1.0;
static bool Arm = false, Armed = false;
static float targetAngleUpdater = 0.0;

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
	vPhi.KpAngular = 0;
	
	vTheta.Kp = 0;
	vTheta.Ki = 0;
	vTheta.Kd = 0;
	vTheta.integratedSum = 0;
	vTheta.previousError = 0;
	vTheta.previousTime = 0;
	vTheta.targetValue = 0;
	vTheta.boundIterm = 0.35;
	vTheta.KpAngular = 0;
	
	vGamma.Kp = 0;
	vGamma.Ki = 0;
	vGamma.Kd = 0;
	vGamma.integratedSum = 0;
	vGamma.previousError = 0;
	vGamma.previousTime = 0;
	vGamma.targetValue = 0;
	vGamma.boundIterm = 0.35;
	vGamma.KpAngular = 0;
	
	vMotor.m1Value = 0;
	vMotor.m2Value = 0;
	vMotor.m3Value = 0;
	vMotor.m4Value = 0;
	vMotor.mMinBound = 1.0;
	vMotor.mMaxBound = 2.2;
	vMotor.mMinPID = -0.5;
	vMotor.mMaxPID = 0.5;
}

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
	if(msg->qID == idConstants[QuadID].value)
	{
		int32_t qSpeed = msg->qThrottle;
		if((qSpeed>=0) & (qSpeed<=120))
			throttle=(((float)qSpeed)/100)+(1.0);
		else
			throttle = 1.0;
		
		int32_t qP = msg->qP;
		if((qP>=0) & (qP<10000))
		{
			vTheta.Kp=((float)qP)/1000;
		}
		
		int32_t qI = msg->qI;
		if((qI>=0) & (qI<1000))
		{
			vTheta.Ki=((float)qI)/1000000;
		}
		if(qI==0)
		{
			vTheta.integratedSum = 0;
		}
		
		int32_t qD = msg->qD;
		if((qD>=0) & (qD<100))
		{
			vTheta.Kd=((float)qD)*1000;
		}
		if(qD==0)
		{
			vTheta.previousError = 0;
		}
		
		int32_t qPA = msg->qPA;
		if((qPA>=0) & (qPA<=100))
		{
			vTheta.KpAngular=((float)qPA)/100;
		}
		
		int32_t qTA = msg->qTargetAngle;
		if((qTA>=-90) & (qTA<=90))
		{
			targetAngleUpdater=((float)qTA*3.14)/180;
		}
		cout<<"T,P,I,D,PA,TA = "<<throttle<<','<<vTheta.Kp<<','<<vTheta.Ki<<','<<vTheta.Kd<<','<<vTheta.KpAngular<<','<<vTheta.targetValue<<"\n";
	}
	else{
		cout<<"Message not for me: "<<msg->qID;
	}
}

int main(int argc, char **argv)
{
	uint64_t timeToCompute = 0,timeToRosPublish = 0, timeToUpdateMotor = 0;
	uint64_t timeToUpdateTargetAngle = 0;
	uint64_t timeToRosSpin = 0, timeNow = 0;
	uint64_t timeSinceArm = 0;
	uint64_t timeToDebugDisplay=0;
	Sensors_init();
	Actuators_init();
	qConfig::readConfigFile("config.txt",timeConstants,7,idConstants,1,filterConstants,2,boolConstants,1);
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
		if(timeNow-timeToUpdateTargetAngle>timeConstants[TIME_TO_UPDATE_TARGETANGLE].value)
		{
			timeToUpdateTargetAngle=timeNow;
			vTheta.targetValue = (filterConstants[M].value*vTheta.targetValue)+(filterConstants[N].value*targetAngleUpdater);
		}
		
		if(timeNow-timeToCompute>timeConstants[TIME_TO_COMPUTE].value)
		{
			timeToCompute = timeNow;
			imud = IMU_data();
			//filterGyroData(&imud);
			//groundDistance = SONAR_data();
			quadController.compute();
		}
		
		if(timeNow-timeToUpdateMotor>timeConstants[TIME_TO_UPDATEMOTOR].value)
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
			if(((timeNow-timeSinceArm)>timeConstants[TIME_TO_ARM].value)&(Armed==true))
			{
				Armed = false;
				break;
			}
		}
		
		if(timeNow-timeToRosSpin>timeConstants[TIME_TO_ROS_SPIN].value)
		{
			timeToRosSpin = timeNow;
			ros::spinOnce();
		}

		if(timeNow-timeToRosPublish>timeConstants[TIME_TO_ROS_PUBLISH].value)
		{
			timeToRosPublish = timeNow;
			quadMsgs::qStatus msg;
			msg.qID = idConstants[QuadID].value;
			msg.qM1 = vMotor.m1Value;
			msg.qM2 = vMotor.m2Value;
			msg.qM3 = vMotor.m3Value;
			msg.qM4 = vMotor.m4Value;
			msg.qXa = imud.fusionPose.x();
			msg.qYa = imud.fusionPose.y();
			msg.qZa = imud.fusionPose.z();
			msg.qXg = imud.gyro.x();
			msg.qYg = imud.gyro.y();
			msg.qZg = imud.gyro.z();
			quadStatus.publish(msg);
		}
		
		if(boolConstants[debugDisplay].value == true)
			if(timeNow-timeToDebugDisplay>timeConstants[TIME_TO_DEBUG_DISPLAY].value)
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
