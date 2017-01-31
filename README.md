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

# Changes
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
