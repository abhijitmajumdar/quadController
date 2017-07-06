#include "actuators.h"
#include "pca9685.h"

#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50
#define PWM_EN 2

using namespace std;
static float M1=0,M2=0,M3=0,M4=0;

int calcTicks(float* impulseMs, int hertz)
{
	float cycleMs = 1000.0f/hertz;
	return (int)(MAX_PWM*(*impulseMs)/cycleMs +0.5f);
}

void Actuators_init(void)
{
	int fd = pca9685Setup(PIN_BASE,0x40,HERTZ);
	pca9685PWMReset(fd);
	wiringPiSetup(); //Not initiated in Sensor even though sensorInit is called first
	pinMode(PWM_EN, OUTPUT);
	pinMode(AMBER, OUTPUT);
	LED_led(AMBER, LOW);
	pinMode(BLUE, OUTPUT);
	LED_led(BLUE, LOW);
}

void PWM_engage(bool enable)
{
	digitalWrite(PWM_EN,!enable);
}

//ch = 0,1,2,3
void PWM_set(int ch, float* millis)
{
	ch += 3;
	pwmWrite(PIN_BASE+ch, calcTicks(millis,HERTZ));
}

void setMotor(float m1, float m2, float m3, float m4)
{
	M1=m1,M2=m2,M3=m3,M4=m4;
	PWM_set(0,&M1);
	PWM_set(1,&M2);
	PWM_set(2,&M3);
	PWM_set(3,&M4);
}

void LED_led(int color, bool state)
{
	digitalWrite(color,!state);
}
