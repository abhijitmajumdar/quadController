# quadController
ROS Controller Node for erle-quadcopter

# How to run the node on raspberry pi
On the raspberry pi use the following command to run the node:
sudo -E bash -c ./bin/quadNode

# The config.txt file
Place a config.txt file in the main directory (quadController folder) with the following contents as an example (Note: each variable is on a new line):

- TIME_TO_UPDATE_TARGETANGLE=100000
- TIME_TO_COMPUTE=5000
- TIME_TO_UPDATEMOTOR=10000
- TIME_TO_ROS_PUBLISH=200000
- TIME_TO_ROS_SPIN=200000
- TIME_TO_ARM=3000000
- TIME_TO_DEBUG_DISPLAY=1000000
- QuadID=4745
- debugDisplay=true
- ANGLE_UPDATE_STEP=0.314

# The RTIMULib.ini file
The RTIMULib makes a configuration file. Make sure of the following things:

IMUType=7
BusIsI2C=false
SPIBus=0
SPISelect=1
SPISpeed=8000000
MPU9250GyroAccelSampleRate=400
MPU9250GyroLpf=1
MPU9250AccelLpf=3
MPU9250GyroFSR=16
MPU9250AccelFSR=16

# Changes
- Made changes to time access to use RTMath time instead of OS time to bring computation down from 250% to 100%
- Made some changes to the configLoader to be more generic
- ROS Publish at 5Hz gives less "C" and "M" time misses
- Added runProgram as a condition when the Compute and MotorUpdate run to shutdown properly
- Added a delay of 3 seconds for threads to close normally before detaching them and exiting program
- The program exits when either the ARM timer runs out or the angle of the quad exceeds 80degrees
