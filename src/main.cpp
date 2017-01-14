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
#include <thread>
#include <sched.h>
#include <unistd.h>
#include <math.h>

using namespace std;

#define TIME_TO_UPDATE_TARGETANGLE 0
#define TIME_TO_COMPUTE 1
#define TIME_TO_UPDATEMOTOR 2
#define TIME_TO_ROS_PUBLISH 3
#define TIME_TO_ROS_SPIN 4
#define TIME_TO_ARM 5
#define TIME_TO_DEBUG_DISPLAY 6
#define QuadID 0
#define debugDisplay 0
#define ANGLE_UPDATE_STEP 0

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
	
fConst angleConstants[1] = {
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
static timespec timeStructure;

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
	clockid_t id=0;
	long prevTime_sec = 0, prevTime_nsec = 0;
	stick_this_thread_to_core(2);
	qControl quadController(&vPhi, &vTheta, &vGamma, &vMotor, &imud, &groundDistance, &throttle);
	
	while(ros::ok())
	{
		IMU_spin();
		clock_gettime(id,&timeStructure);
		long time_sec = timeStructure.tv_sec;
		long time_nsec = timeStructure.tv_nsec;
		long dt = (time_sec-prevTime_sec)*1000000 + (time_nsec-prevTime_nsec)/1000;
		if(dt > timeConstants[TIME_TO_COMPUTE].value)
		{
			imud = IMU_data();
			//groundDistance = SONAR_data();
			quadController.compute();
			prevTime_sec = time_sec;
			prevTime_nsec = time_nsec;
			if(boolConstants[debugDisplay].value)
			{
				if(dt>6000)
					cout<<"C\n";
			}
		}
	}
}

void motorUpdateFunctions()
{
	long prevTime_sec = 0, prevTime_nsec = 0;
	long timeSinceArm_sec = 0, timeSinceArm_nsec = 0;
	stick_this_thread_to_core(3);
	while(ros::ok())
	{
		long time_sec = timeStructure.tv_sec;
		long time_nsec = timeStructure.tv_nsec;
		long dt = (time_sec-prevTime_sec)*1000000 + (time_nsec-prevTime_nsec)/1000;
		if(dt > timeConstants[TIME_TO_UPDATEMOTOR].value)
		{
			if(boolConstants[debugDisplay].value)
			{
				if(dt>12000)
					cout<<"M\n";
			}
			if(Arm == true)
			{
				timeSinceArm_sec = time_sec;
				timeSinceArm_nsec = time_nsec;
				Arm = false;
				Armed = true;
			}
			if(Armed == true)
			{
				setMotor(vMotor.m1Value, vMotor.m2Value, vMotor.m3Value, vMotor.m4Value);
				dt = (time_sec-timeSinceArm_sec)*1000000 + (time_nsec-timeSinceArm_nsec)/1000;
				if((dt > timeConstants[TIME_TO_ARM].value) | (imud.fusionPose.y() > 1.2) | (imud.fusionPose.y() < -1.2))
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
			prevTime_sec = time_sec;
			prevTime_nsec = time_nsec;
		}
	}
}

int main(int argc, char **argv)
{
	long prevTargetAngleTime_sec = 0, prevTargetAngleTime_nsec = 0;
	long prevRosSpinTime_sec = 0, prevRosSpinTime_nsec = 0;
	long prevRosPublishTime_sec = 0, prevRosPublishTime_nsec = 0;
	long dt = 0;
	
	qConfig::readConfigFile("config.txt",timeConstants,7,idConstants,1,angleConstants,1,boolConstants,1);
	Sensors_init();
	Actuators_init();
	initPIDvalues();
	
	LED_led(AMBER,HIGH);
	setMotor(1.0, 1.0, 1.0, 1.0);
	PWM_engage(HIGH);
	ros::init(argc, argv, "quadController");
	ros::NodeHandle n;
	ros::Subscriber quadPosition = n.subscribe("quadPosition", 10, gotquadPosition);
	ros::Subscriber quadArm = n.subscribe("quadArm", 10, gotquadArm);
	ros::Subscriber quadParam = n.subscribe("quadParam", 10, gotquadParam);
	ros::Publisher quadStatus = n.advertise<quadMsgs::qStatus>("quadStatus", 10);
	std::thread computations(computeFunctions);
	std::thread motorupdates(motorUpdateFunctions);
	LED_led(BLUE,HIGH);

	while(ros::ok() & runProgram)
	{
		long time_sec = timeStructure.tv_sec;
		long time_nsec = timeStructure.tv_nsec;
		dt = (time_sec-prevTargetAngleTime_sec)*1000000 + (time_nsec-prevTargetAngleTime_nsec)/1000;
		if(dt > timeConstants[TIME_TO_UPDATE_TARGETANGLE].value)
		{
			float angleDiff = targetAngleUpdater - vTheta.targetValue;
			if(angleDiff != 0)
			{
				if(angleDiff > angleConstants[ANGLE_UPDATE_STEP].value)
					vTheta.targetValue += angleConstants[ANGLE_UPDATE_STEP].value;
				else if(angleDiff < -angleConstants[ANGLE_UPDATE_STEP].value)
					vTheta.targetValue -= angleConstants[ANGLE_UPDATE_STEP].value;
				else
					vTheta.targetValue = targetAngleUpdater;
				cout<<targetAngleUpdater<<"->"<<vTheta.targetValue<<"\n";
			}
			prevTargetAngleTime_sec = time_sec;
			prevTargetAngleTime_nsec = time_nsec;
		}
		dt = (time_sec-prevRosSpinTime_sec)*1000000000 + (time_nsec-prevRosSpinTime_nsec);
		dt = dt/1000;
		if(dt > timeConstants[TIME_TO_ROS_SPIN].value)
		{
			ros::spinOnce();
			prevRosSpinTime_sec = time_sec;
			prevRosSpinTime_nsec = time_nsec;
		}
		dt = (time_sec-prevRosPublishTime_sec)*1000000000 + (time_nsec-prevRosPublishTime_nsec);
		dt = dt/1000;
		if(dt > timeConstants[TIME_TO_ROS_PUBLISH].value)
		{
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
			prevRosPublishTime_sec = time_sec;
			prevRosPublishTime_nsec = time_nsec;
		}
	}
	LED_led(AMBER,LOW);
	motorupdates.detach();
	computations.detach();
	setMotor(1.0, 1.0, 1.0, 1.0);
	//PWM_engage(LOW);
	LED_led(BLUE,LOW);
	return 0;
}
