#include "rcFunctions.h"
#include <iostream>
#include <string>
//#include <thread>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "USBSerial.h"

using namespace std;

#define nMovingAverage 5
#define nbytes 27

static USBSerial *serial;

static int lastTime = 0, dTime = 0;
static float channel[nChannels];
static int lastReadChannel[nChannels][nMovingAverage];
static int lastReadChannelIndexes[nChannels] = {0};
static int counter = 0;

static char rcv[nbytes];
static char getData = '#';
static int rcVal[nChannels];
static bool run = true;
static bool USB_blocking = false;

//static std::thread ReadRCData;

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
				if(lastReadChannelIndexes[counter] >= nMovingAverage) lastReadChannelIndexes[counter] = 0;
			}
			counter++;
		}
}

void RC_PPM_Init(void)
{
	//wiringPiSetup();
	wiringPiISR(PPM_PIN, INT_EDGE_RISING, &pinChange);
}

void getRCData(float *vals)
{
	for(int j=0; j<nChannels;j++)
	{
		channel[j] = 0;
		for(int i=0; i<nMovingAverage; i++)
			channel[j] += lastReadChannel[j][i];
		channel[j] /= (1000*nMovingAverage);
		if(j==CH_THROTTLE) vals[j] = (0.98*vals[j])+(0.02*channel[j]);
		else vals[j] = channel[j];
	}
}

void readUsbSerialData(void)
{
	while(run)
	{
		serial->FlushBuffer();
		serial->Send(getData);
		serial->Send(getData);
		serial->Receive(rcv,nbytes);
		std::string rcvStr = std::string(rcv);
		for(int i=0;i<nChannels;i++)
		{
			if(rcvStr.length()>0)
			{
				rcVal[i] = std::stoi(rcvStr);
				rcvStr.erase(0,5);
			}
		}
		usleep(20000);
	}
}

void RC_USB_init(bool blocking)
{
	serial = new USBSerial("/dev/ttyUSB0",115200);
	usleep(3000000);
	if(!serial->IsOpen())
	{
		run = false;
		std::cout<<"Error opening USB\n";
	}
	USB_blocking = blocking;
	if(USB_blocking == false)
	{
		//ReadRCData = std::thread(readUsbSerialData);
	}
}

void getRCUSBData(float *vals)
{
	if(USB_blocking == true)
	{
		serial->FlushBuffer();
		serial->Send(getData);
		serial->Send(getData);
		serial->Receive(rcv,nbytes);
		std::string rcvStr = std::string(rcv);
		for(int i=0;i<nChannels;i++)
		{
			if(rcvStr.length()>0)
			{
				rcVal[i] = std::stoi(rcvStr);
				rcvStr.erase(0,5);
			}
		}
	}
	for(int i=0;i<nChannels;i++)
	{
		if((rcVal[i]>999) & (rcVal[i]<2001)) vals[i] = float(rcVal[i])/1000;
		else vals[i] = 1.0;
	}
}

void RC_init(bool blocking)
{
	//RC_PPM_Init();
	RC_USB_init(blocking);
}

void RC_deinit()
{
	if(USB_blocking == false)
	{
		//ReadRCData.join();
	}
	serial->Close();
}
