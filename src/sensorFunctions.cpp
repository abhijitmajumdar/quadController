#include "sensorFunctions.h"
#define nMovingAverage 30

using namespace std;

RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
RTIMU *imu = RTIMU::createIMU(settings);

static int lastTime = 0, dTime = 0;
static int channel[nChannels];
static int lastReadChannel[nChannels][nMovingAverage];
static int lastReadChannelIndexes[nChannels] = {0};
static int counter = 0;

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

void pinChange(void)
{
	long tempTime = micros();
	dTime = tempTime - lastTime;
	lastTime = tempTime;
	if(dTime>3000)
		counter = 0;
	else
		if(counter<nChannels)
		{
			if((dTime < 2000) && (dTime > 100))
			{
				lastReadChannel[counter][lastReadChannelIndexes[counter]++] = dTime;
				if(lastReadChannelIndexes[counter] >= nMovingAverage)
					lastReadChannelIndexes[counter] = 0;
			}
			counter++;
		}
}

void RC_PPM_Init(void)
{
	wiringPiSetup();
	wiringPiISR(PPM_PIN, INT_EDGE_RISING, &pinChange);
}

void getRCData(int *vals)
{
	for(int j=0; j<nChannels;j++)
	{
		for(int i=0; i<nMovingAverage; i++)
			channel[j] += lastReadChannel[j][i];
		channel[j] /= nMovingAverage;
		vals[j] = (6*vals[j]+channel[j])/7;
	}
}

void Sensors_init(void)
{
	IMU_init();
	SONAR_init();
	RC_PPM_Init();
}
