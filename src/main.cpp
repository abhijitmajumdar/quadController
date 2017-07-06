// ROS based includes
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include <quadMsgs/qParameters.h>
#include <quadMsgs/qStatus.h>
#include <quadMsgs/qTargets.h>
// System based includes
#include <thread>
#include <sstream>
#include <iostream>
#include <string.h>
#include <sched.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
// User based includes
#include "actuators.h"
#include "sensors.h"
#include "controller.h"
#include "configuration.h"
#include "rcFunctions.h"

using namespace std;

/* Global variables */

// Used for ROS node
static int _argc;
static char **_argv;
static std::string package_path;
// Control variables
static qPIDvariables vPhi,vTheta,vGamma; // vPhi->Roll, vTheta->Pitch, vGamma->Yaw
static qMotorThrust vMotor;
static float throttle = 1.0;
// Program flow control variables
static bool Arm = false, Armed = false, ManualOverride = false, ProgramAlive = true;
static bool ROS_Update = false;
static uint64_t Time;
// Sensor data information
static RTIMU_DATA imud;
static int groundDistance;
static float rcValues[nChannels];
static float rosValues[nChannels];

/* 
 * Used to move thread to selected processor core
 */
int moveThread2Core(int core_id)
{
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
   if (core_id < 0 || core_id >= num_cores) return EINVAL;
   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(core_id, &cpuset);
   pthread_t current_thread = pthread_self();    
   return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}

/*
 * Function handlers to execute safe exit of preogram
 */
void prepare_exit(int s)
{
	ProgramAlive = false;
}

void ctrl_c_exit()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = prepare_exit;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

/*
 * Initialize controller values
 */
void initPIDvalues(void)
{
	vPhi.Kp = 0;
	vPhi.Ki = 0;
	vPhi.Kd = 0;
	vPhi.KpBuffer = qConstants["ROLL_P"];
	vPhi.KiBuffer = qConstants["ROLL_I"];
	vPhi.KdBuffer = qConstants["ROLL_D"];
	vPhi.integratedSum = 0;
	vPhi.previousError = 0;
	vPhi.previousTime = 0;
	vPhi.targetValue = 0;
	vPhi.boundIterm = 0.35;
	vPhi.KpAngular = qConstants["ROLL_PA"];
	
	vTheta.Kp = 0;
	vTheta.Ki = 0;
	vTheta.Kd = 0;
	vTheta.KpBuffer = qConstants["PITCH_P"];
	vTheta.KiBuffer = qConstants["PITCH_I"];
	vTheta.KdBuffer = qConstants["PITCH_D"];
	vTheta.integratedSum = 0;
	vTheta.previousError = 0;
	vTheta.previousTime = 0;
	vTheta.targetValue = 0;
	vTheta.boundIterm = 0.35;
	vTheta.KpAngular = qConstants["PITCH_PA"];
	
	vGamma.Kp = 0;
	vGamma.Ki = 0;
	vGamma.Kd = 0;
	vGamma.KpBuffer = qConstants["YAW_P"];
	vGamma.KiBuffer = qConstants["YAW_I"];
	vGamma.KdBuffer = qConstants["YAW_D"];
	vGamma.integratedSum = 0;
	vGamma.previousError = 0;
	vGamma.previousTime = 0;
	vGamma.targetValue = 0;
	vGamma.boundIterm = 0.35;
	vGamma.KpAngular = qConstants["YAW_PA"];
	
	vMotor.m1Value = 0;
	vMotor.m2Value = 0;
	vMotor.m3Value = 0;
	vMotor.m4Value = 0;
	vMotor.mMinBound = 1.0;
	vMotor.mMaxBound = 2.0;
	vMotor.mMinPID = -0.5;
	vMotor.mMaxPID = 0.5;
}

/*
 * Test function for new features
 */
void new_controller_features(void)
{
	// Tilt-Throttle control
	float cosx = cos(imud.fusionPose.x());
	float cosy = cos(imud.fusionPose.y());
	float cosz = sqrt(abs(1.0 - cosx*cosx - cosy*cosy));
	if(throttle > qConstants["PD_THROTTLE_TRIGGER"]) // Can be done since throttle is updated before calling this fucntion
	{
		// May need to add a proportional constant here as (throttle*P/cosz)
		if(cosz > 0.866) throttle = throttle/cosz; //Only do if tilt is less than 30deg
		else throttle = throttle/0.866;
	}
}

/*
 * Flight Controller: controls flight
 */
void FlightController()
{
	// Move this thread to last core
	moveThread2Core(3);
	uint64_t currentTime = 0;
	uint64_t prevComputeTime = 0;
	uint64_t prevMotorupdateTime = 0;
	uint64_t timeSinceArm = 0;
	// Initialize sensors and actuators
	Sensors_init(&package_path);
	Actuators_init();
	// Initialize parameters and motor
	initPIDvalues();
	qControl quadController(&vPhi, &vTheta, &vGamma, &vMotor, &imud, &groundDistance, &throttle, &qConstants["I_THROTTLE_TRIGGER"], &qConstants["PD_THROTTLE_TRIGGER"]);
	setMotor(1.0, 1.0, 1.0, 1.0);
	if(qConstants["MODE"]==RC_ONLY) ROS_Update = false;
	else ROS_Update = true;
	PWM_engage(HIGH);
	LED_led(BLUE,HIGH);
	
	while(ProgramAlive)
	{
		Time = RTMath::currentUSecsSinceEpoch();
		currentTime = Time;
		
		// Keep updating IMU data
		IMU_spin();
		
		// Time to do computations
		if((currentTime-prevComputeTime) > qConstants["TIME_TO_COMPUTE"])
		{
			// Read IMU data
			imud = IMU_data();
			// Read Sonar data
			//groundDistance = SONAR_data();
			
			// Manual Override is enabled when gear on RC is active(high)
			// Modify the elseif{} later to adjust throttle over ROS also
			if(ManualOverride == true) // If manual overide, get target values from RC
			{
				throttle = rcValues[CH_THROTTLE];
				vTheta.targetValue = -(rcValues[CH_PITCH]-qConstants["CH_PITCH_CALIBRATE"]);
				vPhi.targetValue = (rcValues[CH_ROLL]-qConstants["CH_ROLL_CALIBRATE"]);
				vGamma.targetValue = (rcValues[CH_YAW]-qConstants["CH_YAW_CALIBRATE"]);
			}
			else if(ROS_Update == true)// Else get values from ROS
			{
				throttle = rcValues[CH_THROTTLE];
				vTheta.targetValue = -(rosValues[CH_PITCH]+qConstants["CH_PITCH_CALIBRATE_ROS"]);
				vPhi.targetValue = (rosValues[CH_ROLL]+qConstants["CH_ROLL_CALIBRATE_ROS"]);
				vGamma.targetValue = (rosValues[CH_YAW]+qConstants["CH_YAW_CALIBRATE_ROS"]);
			}
			else
			{
				throttle = 1.0;
				vTheta.targetValue = 0;
				vPhi.targetValue = 0;
				vGamma.targetValue = 0;
			}
			
			// New controller features later to be added into the controller compute functions
			//new_controller_features();
			
			// Perform the calculations
			quadController.compute();
			
			// Debug info
			if(qConstants["debugDisplay"])
			{
				if(currentTime-prevComputeTime > 9000) cout<<"C="<<currentTime-prevComputeTime<<"\n";
			}
			prevComputeTime = currentTime;
		}
		// Time to update the motors/arming
		if((currentTime-prevMotorupdateTime) > qConstants["TIME_TO_UPDATEMOTOR"])
		{
			// If we get an Arm ROS message then ARM the quad and note the time. False the ARM variable to make sure the noted time is not overwritten
			if(Arm == true)
			{
				timeSinceArm = currentTime;
				Arm = false;
				Armed = true;
			}
			
			// RC Gear -> Manual Override / Arming
			if(qConstants["MODE"]==RC_ONLY)
			{
				ManualOverride = true;
				if(rcValues[CH_GEAR]>1.5)
				{
					if(rcValues[CH_THROTTLE]<1.2) Armed = true;
					timeSinceArm = currentTime;
				}
				else Armed = false;
			}
			else
			{
				if(rcValues[CH_GEAR]>1.5) ManualOverride = true;
				else ManualOverride = false;
			}
			
			if(Armed == true) // If Armed, set the motor values as calculated
			{
				setMotor(vMotor.m1Value, vMotor.m2Value, vMotor.m3Value, vMotor.m4Value);
				LED_led(AMBER,HIGH);
				
				// Safety: If no arming msg is received for long OR if quad if at an angle over 70deg pitch/roll then shutdown
				if(((currentTime-timeSinceArm) > qConstants["TIME_TO_ARM"]) | (fabsf(imud.fusionPose.y()) > 1.2) | (fabsf(imud.fusionPose.x()) > 1.2))
				{
					Armed = false;
					setMotor(1.0, 1.0, 1.0, 1.0);
					ProgramAlive = false;
					cout<<"Unarmed\n";
				}
			}
			else // else make sure the motors are not spinning
			{
				setMotor(1.0, 1.0, 1.0, 1.0);
				LED_led(AMBER,LOW);
			}
			
			//Debug info
			if(qConstants["debugDisplay"])
			{
				if(currentTime-prevMotorupdateTime > 15000) cout<<"M="<<currentTime-prevMotorupdateTime<<"\n";
			}
			prevMotorupdateTime = currentTime;
		}
	}
	// Make sure other threads are terminated
	ProgramAlive = false;
	// Set motor throttle to zero
	setMotor(1.0, 1.0, 1.0, 1.0);
	//PWM_engage(LOW);
	LED_led(BLUE,LOW);
}

/*
 * Service ROS Arming message
 */
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

/*
 * Service ROS Target update message
 */
void gotquadTarget(const quadMsgs::qTargets::ConstPtr& msg)
{
	// Make sure to improve the update rate of the TIME_TO_ROS_SPIN in the config file
	// Also need to make sure that if message is not received for a while the ROS_Update becomes false
	if(msg->qID == qConstants["QuadID"])
	{
		rosValues[CH_PITCH] = msg->qPitch;
		rosValues[CH_ROLL] = msg->qRoll;
		rosValues[CH_YAW] = msg->qYaw;
		//rosValues[CH_THROTTLE] = msg->qThrottle;
	}
	else{
		cout<<"Message not for me: "<<msg->qID<<"\n";
	}
}

/*
 * Service ROS Parameters update message
 */
void gotquadParam(const quadMsgs::qParameters::ConstPtr& msg)
{
	if(msg->qID == qConstants["QuadID"])
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

/*
 * Flight Interface: manages interfaces
 */
void FlightInterface()
{
	moveThread2Core(3);
	uint64_t currentTime = 0;
	uint64_t prevRosSpinTime = 0;
	uint64_t prevRosPublishTime = 0;
	uint64_t prevGetRCUSBDataTime = 0;
	ros::Subscriber quadArm;
	ros::Subscriber quadParam;
	ros::Subscriber quadTarget;
	ros::Publisher quadStatus;
	if(qConstants["MODE"]!=RC_ONLY)
	{
		// Setup ROS
		ros::init(_argc, _argv, "quadController");
		ros::NodeHandle n;
		// Setup ROS subscribers and publishers
		quadArm = n.subscribe("quadArm", 10, gotquadArm);
		quadParam = n.subscribe("quadParam", 10, gotquadParam);
		quadTarget = n.subscribe("quadTarget", 10, gotquadTarget);
		quadStatus = n.advertise<quadMsgs::qStatus>("quadStatus", 10);
	}
	// Setup RC
	if(qConstants["MODE"]!=ROS_ONLY) RC_init(true);
	// Set a loop rate so that CPU is not overloaded
	if(qConstants["MODE"]==RC_ONLY) ros::Time::init();
	ros::Rate loop_rate(100);
	
	while(ProgramAlive)
	{
		currentTime = Time;
		if(qConstants["MODE"]!=ROS_ONLY)
		{
			// Time to get RC values from USB
			if((currentTime-prevGetRCUSBDataTime) > qConstants["TIME_TO_GET_RCUSB"])
			{
				getRCUSBData(rcValues);
				prevGetRCUSBDataTime = currentTime;
			}
		}
		if(qConstants["MODE"]!=RC_ONLY)
		{
			// Time to check if we received any ROS messages
			if((currentTime-prevRosSpinTime) > qConstants["TIME_TO_ROS_SPIN"])
			{
				ros::spinOnce();
				prevRosSpinTime = currentTime;
			}
			// Time to send ROS messages
			if((currentTime-prevRosPublishTime) > qConstants["TIME_TO_ROS_PUBLISH"])
			{
				quadMsgs::qStatus msg;
				msg.qID = qConstants["QuadID"];
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
			if(!ros::ok()) ProgramAlive=false;
		}
		loop_rate.sleep();
	}
	// Make sure other threads are terminated
	ProgramAlive = false;
	// Deinitialize the RC
	if(qConstants["MODE"]!=ROS_ONLY) RC_deinit();
}

/*
 * Main body of program
 */
int main(int argc, char **argv)
{
	cout<<"Quad Controller Node\n";
	_argc = argc;
	_argv = argv;
	ctrl_c_exit();
	// Get package path to load configurations
	package_path = ros::package::getPath("quadController");
	// Load configuration parameters
	qConfig::read_config_file(package_path+"/config.txt");
	if(qConstants["debugDisplay"]) qConfig::print_loaded_constants();
	// Create threads
	std::thread flightControl(FlightController);
	std::thread flightInterface(FlightInterface);
	// Wait for threads to finish before exiting
	flightInterface.join();
	flightControl.join();
	cout<<"Exiting\n";
	return 0;
}
