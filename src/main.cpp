#include "ros/ros.h"
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
#include <thread>
#include <sched.h>
#include <unistd.h>

using namespace std;

#define TIME_TO_UPDATE_TARGETANGLE 0
#define TIME_TO_COMPUTE 1
#define TIME_TO_UPDATEMOTOR 2
#define TIME_TO_ROS_PUBLISH 3
#define TIME_TO_ROS_SPIN 4
#define TIME_TO_ARM 5
#define TIME_TO_DEBUG_DISPLAY 6
#define QuadID 7
#define debugDisplay 0
#define ANGLE_UPDATE_STEP 0

lConst longConstants[8] = {
	{"TIME_TO_UPDATE_TARGETANGLE",100000},
	{"TIME_TO_COMPUTE",5000},
	{"TIME_TO_UPDATEMOTOR",10000},
	{"TIME_TO_ROS_PUBLISH",200000},
	{"TIME_TO_ROS_SPIN",200000},
	{"TIME_TO_ARM",3000000},
	{"TIME_TO_DEBUG_DISPLAY",1000000},
	{"QuadID",4745}
	};
	
fConst floatConstants[1] = {
	{"ANGLE_UPDATE_STEP",0.314}
	};
	
bConst boolConstants[1] = {
	{"debugDisplay",false}
	};

static qPIDvariables vPhi,vTheta,vGamma;
static qMotorThrust vMotor;
static RTIMU_DATA imud;
static int groundDistance;
static float throttle = 1.0;
static bool Arm = false, Armed = false, runProgram = true;
static float targetAngleUpdater = 0.0;
static uint64_t Time;

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
	if(msg->qID == longConstants[QuadID].value)
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
			vTheta.Ki=((float)qI)/100000000;
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

int stick_this_thread_to_core(int core_id)
{
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
   if (core_id < 0 || core_id >= num_cores)
      return EINVAL;

   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(core_id, &cpuset);

   pthread_t current_thread = pthread_self();    
   return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}

void computeFunctions()
{
	uint64_t currentTime = 0;
	uint64_t prevComputeTime = 0;
	stick_this_thread_to_core(3);
	qControl quadController(&vPhi, &vTheta, &vGamma, &vMotor, &imud, &groundDistance, &throttle);
	
	while(ros::ok() & runProgram)
	{
		Time = RTMath::currentUSecsSinceEpoch();
		currentTime = Time;
		IMU_spin();
		if((currentTime-prevComputeTime) > longConstants[TIME_TO_COMPUTE].value)
		{
			imud = IMU_data();
			//groundDistance = SONAR_data();
			quadController.compute();
			if(boolConstants[debugDisplay].value)
			{
				if(currentTime-prevComputeTime > 6000)
					cout<<"C="<<currentTime-prevComputeTime<<"\n";
			}
			prevComputeTime = Time;
		}
	}
}

void motorUpdateFunctions()
{
	uint64_t currentTime = 0;
	uint64_t prevMotorupdateTime = 0;
	uint64_t timeSinceArm = 0;
	stick_this_thread_to_core(2);
	while(ros::ok() & runProgram)
	{
		currentTime = Time;
		if((currentTime-prevMotorupdateTime) > longConstants[TIME_TO_UPDATEMOTOR].value)
		{
			if(Arm == true)
			{
				timeSinceArm = currentTime;
				Arm = false;
				Armed = true;
			}
			if(Armed == true)
			{
				setMotor(vMotor.m1Value, vMotor.m2Value, vMotor.m3Value, vMotor.m4Value);
				if(((currentTime-timeSinceArm) > longConstants[TIME_TO_ARM].value) | (imud.fusionPose.y() > 1.2) | (imud.fusionPose.y() < -1.2))
				{
					Armed = false;
					setMotor(1.0, 1.0, 1.0, 1.0);
					runProgram = false;
					cout<<"Unarmed\n";
				}
			}
			else
			{
				setMotor(1.0, 1.0, 1.0, 1.0);
			}
			
			if(boolConstants[debugDisplay].value)
			{
				if(currentTime-prevMotorupdateTime > 12000)
					cout<<"M="<<currentTime-prevMotorupdateTime<<"\n";
			}
			prevMotorupdateTime = currentTime;
		}
	}
}

int main(int argc, char **argv)
{
	uint64_t currentTime = 0;
	uint64_t prevTargetAngleTime = 0;
	uint64_t prevRosSpinTime = 0;
	uint64_t prevRosPublishTime = 0;
	
	qConfig::readConfigFile("config.txt",longConstants,8,floatConstants,1,boolConstants,1);
	Sensors_init();
	Actuators_init();
	initPIDvalues();
	
	LED_led(AMBER,HIGH);
	setMotor(1.0, 1.0, 1.0, 1.0);
	PWM_engage(HIGH);
	ros::init(argc, argv, "quadController");
	ros::NodeHandle n;
	ros::Subscriber quadArm = n.subscribe("quadArm", 10, gotquadArm);
	ros::Subscriber quadParam = n.subscribe("quadParam", 10, gotquadParam);
	ros::Publisher quadStatus = n.advertise<quadMsgs::qStatus>("quadStatus", 10);
	std::thread computations(computeFunctions);
	std::thread motorupdates(motorUpdateFunctions);
	ros::Rate loop_rate(20);
	LED_led(BLUE,HIGH);

	while(ros::ok() & runProgram)
	{
		currentTime = Time;
		if((currentTime-prevTargetAngleTime) > longConstants[TIME_TO_UPDATE_TARGETANGLE].value)
		{
			float angleDiff = targetAngleUpdater - vTheta.targetValue;
			if(angleDiff != 0)
			{
				if(angleDiff > floatConstants[ANGLE_UPDATE_STEP].value)
					vTheta.targetValue += floatConstants[ANGLE_UPDATE_STEP].value;
				else if(angleDiff < -floatConstants[ANGLE_UPDATE_STEP].value)
					vTheta.targetValue -= floatConstants[ANGLE_UPDATE_STEP].value;
				else
					vTheta.targetValue = targetAngleUpdater;
				cout<<targetAngleUpdater<<"->"<<vTheta.targetValue<<"\n";
			}
			prevTargetAngleTime = currentTime;
		}
		if((currentTime-prevRosSpinTime) > longConstants[TIME_TO_ROS_SPIN].value)
		{
			ros::spinOnce();
			prevRosSpinTime = currentTime;
		}
		if((currentTime-prevRosPublishTime) > longConstants[TIME_TO_ROS_PUBLISH].value)
		{
			quadMsgs::qStatus msg;
			msg.qID = longConstants[QuadID].value;
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
			prevRosPublishTime = currentTime;
		}
		loop_rate.sleep();
	}
	LED_led(AMBER,LOW);
	cout<<"Exiting\n";
	runProgram = false;
	for(int i=60;i--;i>0)
		loop_rate.sleep();
	motorupdates.detach();
	computations.detach();
	setMotor(1.0, 1.0, 1.0, 1.0);
	//PWM_engage(LOW);
	LED_led(BLUE,LOW);
	return 0;
}
