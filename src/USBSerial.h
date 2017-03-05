#ifndef _USB_SERIAL
#define _USB_SERIAL

#include <string>


class  USBSerial
{

public:

  int handle;
  std::string  deviceName;
  int baud;

  USBSerial(std::string deviceName, int baud);
  ~USBSerial();

  bool Send( unsigned char  * data,int len);
  bool Send(unsigned char value);
  bool Send( std::string value);
  int Receive( char  * data, int len);
  bool IsOpen(void);
  void Close(void);
  bool Open(std::string deviceName, int baud);
  bool NumberByteRcv(int &bytelen);
  void FlushBuffer(void);
};

#endif //_USB_SERIAL
