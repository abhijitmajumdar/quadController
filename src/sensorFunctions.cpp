#include "sensorFunctions.h"

using namespace std;

RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
RTIMU *imu = RTIMU::createIMU(settings);

void IMU_init(void)
{
	imu->IMUInit();
	imu->setSlerpPower(0.02);
	imu->setGyroEnable(true);
	imu->setAccelEnable(true);
	imu->setCompassEnable(true);
}

void IMU_spin(void)
{
	imu->IMURead();
}

RTIMU_DATA IMU_data(void)
{
	return imu->getIMUData();
}

volatile int fd=-1;
void SONAR_init(void)
{
        fd = serialOpen("/dev/ttyAMA0",9600);
        if(fd<0)
                cout<<"Error\n";
        cout<<fd<<"\n";
        //wiringPiSetup();
}

int SONAR_data(void)
{
        bool findData = true;
        char data[3];
        serialFlush(fd);
        while(findData == true)
        {
                if(serialDataAvail(fd))
                {
                        if(serialGetchar(fd)=='R')
                        {
                                while(serialDataAvail(fd)==0);
                                data[0] = serialGetchar(fd);
                                while(serialDataAvail(fd)==0);
                                data[1] = serialGetchar(fd);
				while(serialDataAvail(fd)==0);
                                data[2] = serialGetchar(fd);
                                findData = false;
                        }
                }
        }
        return ((data[0]-48)*100)+((data[1]-48)*10)+(data[2]-48);
}

void Sensors_init(void)
{
	IMU_init();
	SONAR_init();
}
