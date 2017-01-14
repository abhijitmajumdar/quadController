# quadController
ROS Controller Node for erle-quadcopter

# How to run the node on raspberry pi
On the raspberry pi use the following command to run the node:
sudo -E bash -c ./bin/quadNode

# The config.txt file
Place a config.txt file in the main directory (quadController folder) with the following contents as an example (Note: each variable is on a new line):

TIME_TO_UPDATE_TARGETANGLE=80000
TIME_TO_COMPUTE=5000
TIME_TO_UPDATEMOTOR=10000
TIME_TO_ROS_PUBLISH=200000
TIME_TO_ROS_SPIN=200000
TIME_TO_ARM=3000000
TIME_TO_DEBUG_DISPLAY=1000000
QuadID=4745
debugDisplay=false
ANGLE_UPDATE_STEP=0.314

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
- Made seperate threads(with affinity) for compute functions and motor update functions
- Changed angle update to step update and its corresponding variables in config.txt
- computes now at 200Hz
- Added debug on/off for displaying when compute/motorUpdate take longer time
- Added safety feature where if quad goes beyond angle 70degree, it shuts down
