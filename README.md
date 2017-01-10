# quadController
ROS Controller Node for erle-quadcopter

# How to run the node on raspberry pi
On the raspberry pi use the following command to run the node:
sudo -E bash -c ./bin/quadNode

# The config.txt file
Place a config.txt file in the main directory (quadController folder) with the following contents as an example (Note: each variable is on a new line):

TIME_TO_UPDATE_TARGETANGLE=80000
TIME_TO_COMPUTE=20000
TIME_TO_UPDATEMOTOR=20000
TIME_TO_ROS_PUBLISH=200000
TIME_TO_ROS_SPIN=200000
TIME_TO_ARM=3000000
TIME_TO_DEBUG_DISPLAY=1000000
QuadID=4745
M=0.9
N=0.1
debugDisplay=false
