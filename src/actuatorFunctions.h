#ifndef _ACTUATORFUNCTIONS_H
#define _ACTUATORFUNCTIONS_H

#include <iostream>
#include <signal.h>
#include <wiringPi.h>

#define AMBER 5
#define BLUE 6

void Actuators_init(void);
void PWM_engage(bool enable);
void PWM_set(int ch, float* millis);
void setMotor(float m1, float m2, float m3, float m4);
void LED_led(int color, bool state);

#endif //_ACTUATORFUNCTIONS_H
