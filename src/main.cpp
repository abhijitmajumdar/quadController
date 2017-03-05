#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <quadMsgs/qParameters.h>
#include <quadMsgs/qStatus.h>
#include <quadMsgs/qTargets.h>
#include <thread>
#include <sstream>
#include <iostream>
#include <string.h>
#include <sched.h>
#include <unistd.h>
#include <math.h>
#include "actuatorFunctions.h"
#include "sensorFunctions.h"
#include "controller.h"
#include "configLoader.h"
#include "rcFunctions.h"

using namespace std;

enum{
	TIME_TO_UPDATE_TARGETANGLE = 0,
	TIME_TO_COMPUTE,
	TIME_TO_UPDATEMOTOR,
	TIME_TO_ROS_PUBLISH,
	TIME_TO_ROS_SPIN,
	TIME_TO_ARM,
	TIME_TO_DEBUG_DISPLAY,
	TIME_TO_GET_RCUSB,
	QuadID,
	NUM_LONG_CFG_VARIABLES
};
std::map<int,lConst> longConstants = {
	{TIME_TO_UPDATE_TARGETANGLE,{"TIME_TO_UPDATE_TARGETANGLE",100000}},
	{TIME_TO_COMPUTE,{"TIME_TO_COMPUTE",5000}},
	{TIME_TO_UPDATEMOTOR,{"TIME_TO_UPDATEMOTOR",10000}},
	{TIME_TO_ROS_PUBLISH,{"TIME_TO_ROS_PUBLISH",200000}},
	{TIME_TO_ROS_SPIN,{"TIME_TO_ROS_SPIN",200000}},
	{TIME_TO_ARM,{"TIME_TO_ARM",3000000}},
	{TIME_TO_DEBUG_DISPLAY,{"TIME_TO_DEBUG_DISPLAY",1000000}},
	{TIME_TO_GET_RCUSB,{"TIME_TO_GET_RCUSB",20000}},
	{QuadID,{"QuadID",4745}}
};

enum{
	I_THROTTLE_TRIGGER = 0,
	PD_THROTTLE_TRIGGER,
	YAW_PA,
	YAW_P,
	YAW_I,
	YAW_D,
	ROLL_PA,
	ROLL_P,
	ROLL_I,
	ROLL_D,
	PITCH_PA,
	PITCH_P,
	PITCH_I,
	PITCH_D,
	ANGLE_UPDATE_STEP,
	CH_THROTTLE_CALIBRATE,
	CH_PITCH_CALIBRATE,
	CH_ROLL_CALIBRATE,
	CH_YAW_CALIBRATE,
	CH_SWC_CALIBRATE,
	CH_SWB_CALIBRATE,
	NUM_FLOAT_CFG_VARIABLES
};
std::map<int,fConst> floatConstants = {
	{I_THROTTLE_TRIGGER,{"I_THROTTLE_TRIGGER",1.5}},
	{PD_THROTTLE_TRIGGER,{"PD_THROTTLE_TRIGGER",1.3}},
	{YAW_PA,{"YAW_PA",2.0}},
	{YAW_P,{"YAW_P",0}},
	{YAW_I,{"YAW_I",0}},
	{YAW_D,{"YAW_D",0}},
	{ROLL_PA,{"ROLL_PA",5.0}},
	{ROLL_P,{"ROLL_P",0}},
	{ROLL_I,{"ROLL_I",0}},
	{ROLL_D,{"ROLL_D",0}},
	{PITCH_PA,{"PITCH_PA",5.0}},
	{PITCH_P,{"PITCH_P",0}},
	{PITCH_I,{"PITCH_I",0}},
	{PITCH_D,{"PITCH_D",0}},
	{ANGLE_UPDATE_STEP,{"ANGLE_UPDATE_STEP",0.314}},
	{CH_THROTTLE_CALIBRATE,{"CH_THROTTLE_CALIBRATE",1.0}},
	{CH_PITCH_CALIBRATE,{"CH_PITCH_CALIBRATE",1.5}},
	{CH_ROLL_CALIBRATE,{"CH_ROLL_CALIBRATE",1.5}},
	{CH_YAW_CALIBRATE,{"CH_YAW_CALIBRATE",1.5}},
	{CH_SWC_CALIBRATE,{"CH_SWC_CALIBRATE",1.0}},
	{CH_SWB_CALIBRATE,{"CH_SWB_CALIBRATE",1.0}}
};

enum{
	doTargetAngleUpdate = 0,
	debugDisplay,
	NUM_BOOL_CFG_VARIABLES
};
std::map<int,bConst> boolConstants = {
	{doTargetAngleUpdate,{"doTargetAngleUpdate",false}},
	{debugDisplay,{"debugDisplay",false}}
};

static int _argc;
static char **_argv;
static qPIDvariables vPhi,vTheta,vGamma; // vPhi->Roll, vTheta->Pitch, vGamma->Yaw
static qMotorThrust vMotor;
static RTIMU_DATA imud;
static int groundDistance;
static float throttle = 1.0;
static bool Arm = false, Armed = false, ManualOverride = false, runProgram = true;
static float targetAngleUpdater = 0.0;
static uint64_t Time;
static float rcValues[nChannels];

int moveThread2Core(int core_id)
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

void initPIDvalues(void)
{
	vPhi.Kp = 0;
	vPhi.Ki = 0;
	vPhi.Kd = 0;
	vPhi.KpBuffer = floatConstants[ROLL_P].value;
	vPhi.KiBuffer = floatConstants[ROLL_I].value;
	vPhi.KdBuffer = floatConstants[ROLL_D].value;
	vPhi.integratedSum = 0;
	vPhi.previousError = 0;
	vPhi.previousTime = 0;
	vPhi.targetValue = 0;
	vPhi.boundIterm = 0.35;
	vPhi.KpAngular = floatConstants[ROLL_PA].value;
	
	vTheta.Kp = 0;
	vTheta.Ki = 0;
	vTheta.Kd = 0;
	vTheta.KpBuffer = floatConstants[PITCH_P].value;
	vTheta.KiBuffer = floatConstants[PITCH_I].value;
	vTheta.KdBuffer = floatConstants[PITCH_D].value;
	vTheta.integratedSum = 0;
	vTheta.previousError = 0;
	vTheta.previousTime = 0;
	vTheta.targetValue = 0;
	vTheta.boundIterm = 0.35;
	vTheta.KpAngular = floatConstants[PITCH_PA].value;
	
	vGamma.Kp = 0;
	vGamma.Ki = 0;
	vGamma.Kd = 0;
	vGamma.KpBuffer = floatConstants[YAW_P].value;
	vGamma.KiBuffer = floatConstants[YAW_I].value;
	vGamma.KdBuffer = floatConstants[YAW_D].value;
	vGamma.integratedSum = 0;
	vGamma.previousError = 0;
	vGamma.previousTime = 0;
	vGamma.targetValue = 0;
	vGamma.boundIterm = 0.35;
	vGamma.KpAngular = floatConstants[YAW_PA].value;
	
	vMotor.m1Value = 0;
	vMotor.m2Value = 0;
	vMotor.m3Value = 0;
	vMotor.m4Value = 0;
	vMotor.mMinBound = 1.0;
	vMotor.mMaxBound = 2.0;
	vMotor.mMinPID = -0.5;
	vMotor.mMaxPID = 0.5;
}

void FlightController()
{
	uint64_t currentTime = 0;
	uint64_t prevComputeTime = 0;
	uint64_t prevMotorupdateTime = 0;
	uint64_t timeSinceArm = 0;
	// Move this thread to last core
	moveThread2Core(3);
	// Initialize sensors and actuators
	Sensors_init();
	Actuators_init();
	// Initialize parameters and motor
	initPIDvalues();
	qControl quadController(&vPhi, &vTheta, &vGamma, &vMotor, &imud, &groundDistance, &throttle, &floatConstants[I_THROTTLE_TRIGGER].value, &floatConstants[PD_THROTTLE_TRIGGER].value);
	setMotor(1.0, 1.0, 1.0, 1.0);
	PWM_engage(HIGH);
	LED_led(BLUE,HIGH);
	
	while(runProgram)
	{
		Time = RTMath::currentUSecsSinceEpoch();
		currentTime = Time;
		
		// Keep updating IMU data
		IMU_spin();
		
		// Time to do computations
		if((currentTime-prevComputeTime) > longConstants[TIME_TO_COMPUTE].value)
		{
			// Read IMU data
			imud = IMU_data();
			// Read Sonar data
			//groundDistance = SONAR_data();
			
			if(ManualOverride == true) // If manual overide, get target values from RC
			{
				throttle = rcValues[CH_THROTTLE];
				vTheta.targetValue = -(rcValues[CH_PITCH]-floatConstants[CH_PITCH_CALIBRATE].value);
				vPhi.targetValue = (rcValues[CH_ROLL]-floatConstants[CH_ROLL_CALIBRATE].value);
				vGamma.targetValue = (rcValues[CH_YAW]-floatConstants[CH_YAW_CALIBRATE].value);
			}
			else // Else get values from ROS
			{
				throttle = 1.0;
				vTheta.targetValue = 0;
				vPhi.targetValue = 0;
				vGamma.targetValue = 0;
			}
			
			// Perform the calculations
			quadController.compute();
			
			// Debug info
			if(boolConstants[debugDisplay].value)
			{
				if(currentTime-prevComputeTime > 9000) cout<<"C="<<currentTime-prevComputeTime<<"\n";
			}
			prevComputeTime = currentTime;
		}
		// Time to update the motors/arming
		if((currentTime-prevMotorupdateTime) > longConstants[TIME_TO_UPDATEMOTOR].value)
		{
			// If we get an Arm ROS message then ARM the quad and note the time. False the ARM variable to make sure the noted time is not overwritten
			if(Arm == true)
			{
				timeSinceArm = currentTime;
				Arm = false;
				Armed = true;
			}
			
			// RC Gear -> Manual Override
			if(rcValues[CH_GEAR]>1.5)
			{
				if((ManualOverride==false)&(rcValues[CH_THROTTLE]<1.2)) ManualOverride = true;
			}
			else ManualOverride = false;
			
			if(Armed == true) // If Armed, set the motor values as calculated
			{
				setMotor(vMotor.m1Value, vMotor.m2Value, vMotor.m3Value, vMotor.m4Value);
				LED_led(AMBER,HIGH);
				
				// Safety: If no arming msg is received for long OR if quad if at an angle over 70deg pitch/roll then shutdown
				if(((currentTime-timeSinceArm) > longConstants[TIME_TO_ARM].value) | (fabsf(imud.fusionPose.y()) > 1.2) | (fabsf(imud.fusionPose.x()) > 1.2))
				{
					Armed = false;
					setMotor(1.0, 1.0, 1.0, 1.0);
					runProgram = false;
					cout<<"Unarmed\n";
				}
			}
			else // else make sure the motors are not spinning
			{
				setMotor(1.0, 1.0, 1.0, 1.0);
				LED_led(AMBER,LOW);
			}
			
			//Debug info
			if(boolConstants[debugDisplay].value)
			{
				if(currentTime-prevMotorupdateTime > 15000) cout<<"M="<<currentTime-prevMotorupdateTime<<"\n";
			}
			prevMotorupdateTime = currentTime;
		}
	}
	// Make sure other threads are terminated
	runProgram = false;
	// Set motor throttle to zero
	setMotor(1.0, 1.0, 1.0, 1.0);
	//PWM_engage(LOW);
	LED_led(BLUE,LOW);
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

void gotquadTarget(const quadMsgs::qTargets::ConstPtr& msg)
{
	if(msg->qID == longConstants[QuadID].value)
	{
		float qPitch = msg->qPitch;
		float qRoll = msg->qRoll;
		float qYaw = msg->qYaw;
		float qThrottle = msg->qThrottle;
	}
	else{
		cout<<"Message not for me: "<<msg->qID<<"\n";
	}
}

void gotquadParam(const quadMsgs::qParameters::ConstPtr& msg)
{
	if(msg->qID == longConstants[QuadID].value)
	{
		int32_t qP = msg->qP;
		int32_t qI = msg->qI;
		int32_t qD = msg->qD;
		int32_t qPA = msg->qPA;
		if((qP>=0) & (qP<10000))
		{
			vTheta.KpBuffer = ((float)qP)/1000;
			vPhi.KpBuffer = vTheta.KpBuffer;
		}
		if((qI>=0) & (qI<1000))
		{
			vTheta.KiBuffer = ((float)qI)/100000000;
			vPhi.KiBuffer = vTheta.KiBuffer;
		}
		if((qD>=0) & (qD<100))
		{
			vTheta.KdBuffer = ((float)qD)*100;
			vPhi.KdBuffer = vTheta.KdBuffer;
		}		
		cout<<"P,I,D,PA = "<<vTheta.Kp<<','<<vTheta.Ki<<','<<vTheta.Kd<<','<<vTheta.KpAngular<<"\n";
	}
	else{
		cout<<"Message not for me: "<<msg->qID<<"\n";
	}
}

void FlightInterface()
{
	uint64_t currentTime = 0;
	uint64_t prevTargetAngleTime = 0;
	uint64_t prevRosSpinTime = 0;
	uint64_t prevRosPublishTime = 0;
	uint64_t prevGetRCUSBDataTime = 0;
	moveThread2Core(3);
	// Setup ROS
	ros::init(_argc, _argv, "quadController");
	ros::NodeHandle n;
	// Setup ROS subscribers and publishers
	ros::Subscriber quadArm = n.subscribe("quadArm", 10, gotquadArm);
	ros::Subscriber quadParam = n.subscribe("quadParam", 10, gotquadParam);
	ros::Subscriber quadTarget = n.subscribe("quadTarget", 10, gotquadTarget);
	ros::Publisher quadStatus = n.advertise<quadMsgs::qStatus>("quadStatus", 10);
	// Set a loop rate so that CPU is not overloaded
	ros::Rate loop_rate(100);
	// Setup RC signals
	RC_init(true);
	
	while(ros::ok() & runProgram)
	{
		currentTime = Time;
		// Time to get RC values from USB
		if((currentTime-prevGetRCUSBDataTime) > longConstants[TIME_TO_GET_RCUSB].value)
		{
			getRCUSBData(rcValues);
		}
		// Time to update the target angels in steps
		if((currentTime-prevTargetAngleTime) > longConstants[TIME_TO_UPDATE_TARGETANGLE].value)
		{
			if(boolConstants[doTargetAngleUpdate].value==true)
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
			}
			prevTargetAngleTime = currentTime;
		}
		// Time to check if we received any ROS messages
		if((currentTime-prevRosSpinTime) > longConstants[TIME_TO_ROS_SPIN].value)
		{
			ros::spinOnce();
			prevRosSpinTime = currentTime;
		}
		// Time to send ROS messages
		if((currentTime-prevRosPublishTime) > longConstants[TIME_TO_ROS_PUBLISH].value)
		{
			quadMsgs::qStatus msg;
			msg.qID = longConstants[QuadID].value;
			msg.qM1 = vMotor.m1Value;
			msg.qM2 = vMotor.m2Value;
			msg.qM3 = vMotor.m3Value;
			msg.qM4 = vMotor.m4Value;
			msg.qXp = imud.fusionPose.x();
			msg.qYp = imud.fusionPose.y();
			msg.qZp = imud.fusionPose.z();
			msg.qXa = imud.accel.x();
			msg.qYa = imud.accel.y();
			msg.qZa = imud.accel.z();
			msg.qXg = imud.gyro.x();
			msg.qYg = imud.gyro.y();
			msg.qZg = imud.gyro.z();
			msg.qXm = imud.compass.x();
			msg.qYm = imud.compass.y();
			msg.qZm = imud.compass.z();
			quadStatus.publish(msg);
			prevRosPublishTime = currentTime;
		}
		loop_rate.sleep();
	}
	// Make sure other threads are terminated
	runProgram = false;
	// Deinitialize the RC
	RC_deinit();
}

int main(int argc, char **argv)
{
	cout<<"Quad Controller Node\n";
	_argc = argc;
	_argv = argv;
	// Load configuration parameters
	qConfig::readConfigFile("config.txt",longConstants,NUM_LONG_CFG_VARIABLES,floatConstants,NUM_FLOAT_CFG_VARIABLES,boolConstants,NUM_BOOL_CFG_VARIABLES);
	std::thread flightControl(FlightController);
	std::thread flightInterface(FlightInterface);
	flightInterface.join();
	flightControl.join();
	cout<<"Exiting\n";
	return 0;
}
