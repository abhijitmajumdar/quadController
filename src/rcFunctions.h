#ifndef _RCFUNCTIONS_H
#define _RCFUNCTIONS_H

#define FSTH9X

#ifdef TGYIA6
// On the TGY-iA6 receiver
#define CH_ROLL 0 // CH1: Left-Right/Roll
#define CH_PITCH 1 // CH2: Up-Down/Pitch
#define CH_THROTTLE 2 // CH3: Throttle
#define CH_YAW 3 // CH4: Yaw
#define CH_SWC 4 // CH5: SWC
#define CH_SWB 5 // CH6: SWB

#define nChannels 6
#endif

#ifdef FSTH9X
// On the FS-TH9X receiver
#define CH_ROLL 0 // CH1: Left-Right/Roll
#define CH_PITCH 1 // CH2: Up-Down/Pitch
#define CH_THROTTLE 2 // CH3: Throttle
#define CH_YAW 3 // CH4: Yaw
#define CH_GEAR 4 // CH5: Gear

#define nChannels 5
#endif

#define PPM_PIN 7

void pinChange(void);
void RC_PPM_Init(void);
void getRCData(float *vals);
void RC_USB_init(bool blocking);
void readUsbSerialData(void);
void getRCUSBData(float *vals);
void RC_init(bool blocking);
void RC_deinit();

#endif //_RCFUNCTIONS_H
