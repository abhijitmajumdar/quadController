# quadController
ROS Controller Node

![Alt text](/Photos/bigquad.jpg?raw=true "The larger quadcopter")
![Alt text](/Photos/flight.png?raw=true "In flight")

# Compiling
Compile using rosmake. It should be noted that the code uses RTIMULib and pca9685 libraries for interfacing the sensors and actuators, which must be installed before compiling this code
- https://github.com/RTIMULib/RTIMULib
- https://github.com/TeraHz/PCA9685

# How to run the node on raspberry pi
On the raspberry pi use the following command to run the node:
sudo -E bash -c ./bin/quadNode

# The config.txt file
Place a config.txt file in the main directory (quadController folder) with the following contents as an example (Note: each variable is on a new line):

- TIME_TO_UPDATE_TARGETANGLE=100000
- TIME_TO_COMPUTE=5000
- TIME_TO_UPDATEMOTOR=10000
- TIME_TO_ROS_PUBLISH=100000
- TIME_TO_ROS_SPIN=200000
- TIME_TO_ARM=3000000
- TIME_TO_DEBUG_DISPLAY=1000000
- TIME_TO_GET_RCUSB=20000
- QuadID=4745
- debugDisplay=false
- ANGLE_UPDATE_STEP=0.314
- doTargetAngleUpdate=false
- CH_THROTTLE_CALIBRATE=1.0
- CH_PITCH_CALIBRATE=1.5
- CH_ROLL_CALIBRATE=1.5
- CH_YAW_CALIBRATE=1.5
- CH_SWC_CALIBRATE=1.0
- CH_SWB_CALIBRATE=1.0
- I_THROTTLE_TRIGGER=1.5
- PD_THROTTLE_TRIGGER=1.3
- YAW_PA=2.0
- YAW_P=0.4
- YAW_I=0
- YAW_D=0
- ROLL_PA=5.2
- ROLL_P=0.05
- ROLL_I=0.00000001
- ROLL_D=0
- PITCH_PA=5.2
- PITCH_P=0.05
- PITCH_I=0.00000001
- PITCH_D=0


# The RTIMULib.ini file
The RTIMULib makes a configuration file. Make sure of the following things:

- IMUType=7
- BusIsI2C=false
- SPIBus=0
- SPISelect=1
- SPISpeed=8000000
- MPU9250GyroAccelSampleRate=400
- MPU9250GyroLpf=1
- MPU9250AccelLpf=3
- MPU9250GyroFSR=16
- MPU9250AccelFSR=16

# I2C changes on the RPi
Make sure the I2C is at 400Khz which makes updates to the motors in ~1ms.
Make the following changes to the file /boot/config.txt
- dtparam=i2c1_arm=on
- dtparam=i2c1_baudrate=400000

# Bootloader changes to modify the Kernel use on RPi
The following will restrict the kernel to use only the first 3 cpu for general tasks, leaving the 4th one only for precise quad computations
In the file /boot/cmdline.txt add the following:
isolcpus=3
Also change the affinity for the IRQ in the system. This can be done more thoroughly by modifying the /proc/irq/IRQ_NUMBER/smp_affinity file of each IRQ.
modify the file /proc/irq/default_smp_affinity from "f" to "7" to use only the first 3 cpus

# Design
The STL files for the 3D models of custom printed parts is available in the Design Files folder. Pictures of the models for reference can be found in the Photos directory.
The 3D parts of the smaller quadcopter are used from a Thingiverse project "T4 Quadcopter Mini 315 (7-8 inch props)"
- http://www.thingiverse.com/thing:408363

# Changes
- Added seperate file for rcReceiver functions, and files for associated USB serial read functions
- Added more config parameters for all controller constants and throttle triggers (I and PD)
- made some changes to the config loader
- moved throttle trigger to controller.cpp and added pointers to thriggers in the contructor
- added buffer values for PID in the structure to store values received from ROS or set values from config temporarily untill trigger is used
- added qTarget msg type to seperate status, arm, parameter and target values
- added static variables argv and argc to use while ros initialization in a different function
- added a manual override feature (and variable)
- changed name to moveThread2Core
- added more status parameters
- restructured the entire main program to do config in main followed by FlightController and FlightInterface threads
- made some changes to the sensor initializations
- added feature in RC_init() to configure it as blocking or non-blocking using a seperate thread

- Made changes to the configLoader. To add a new variable now just modify the top of the main file.
- Changed rc-data from int to float and hence also the config file
- Corrected moving average filter for rc receiver
- removed complementry filter, since it did not remove the problematic spikes and cause delayed response
- reduced moving average filter set to 5
- added I triggers for pitch,roll and yaw

- Added RC-input libraries, modified the throttle, pitch, roll and yaw to take inputs from RC
- Changed the exit procedure to thread.join()
- Combined the compute and motorUpdate threads to one
	- which might be the reason to cause the sudden jerks in the motor (tests show it was due to test bed)
	- this reduced the cpu usage drastically, since the motorupdate was taking up 100%
	- overall usage by compute and motor is just ~30%
- Changed the cpu affinity of this combined thread to cpu3 
- made changes to the kernal to use only the 1st 3 cpu, leaving the 4th-cpu3 only for the above
- added calibration constants for rc inputs

- Made changes to time access to use RTMath time instead of OS time to bring computation down from 250% to 100%
- Made some changes to the configLoader to be more generic
- ROS Publish at 5Hz gives less "C" and "M" time misses
- Added runProgram as a condition when the Compute and MotorUpdate run to shutdown properly
- Added a delay of 3 seconds for threads to close normally before detaching them and exiting program
- The program exits when either the ARM timer runs out or the angle of the quad exceeds 80degrees
- Changed the I2C clock to change the setMotor() time from 3.5ms to 1ms
