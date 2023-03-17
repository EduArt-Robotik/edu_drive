#include "IOTAdapter.h"
#include <iostream>
#include <sys/time.h>
#include <unistd.h>

namespace edu
{

void byteArrayToFloat(int8_t* byteArray, float* floatVar)
{
	uint32_t* var = (uint32_t*)floatVar;
	*var          = byteArray[0] << 24;
	*var         |= byteArray[1] << 16;
	*var         |= byteArray[2] << 8;
	*var         |= byteArray[3];
}

void floatToByteArray(float* floatVar, int8_t* byteArray)
{
   uint8_t* var = (uint8_t*)(floatVar);
   memcpy(byteArray, var, sizeof(floatVar));
}

void intToByteArray(uint32_t* iVar, int8_t* byteArray)
{
   uint8_t* var = (uint8_t*)(iVar);
   memcpy(byteArray, var, sizeof(iVar));
}

const char* devPath = "/dev/ttyS3";

IOTAdapter::IOTAdapter()
{
   _timeCom = 0.0;
   _q.resize(4);
   _uart = new SerialPort(devPath, B115200);
}
   
IOTAdapter::~IOTAdapter()
{
   delete _uart;
}

char* initCode = "EduArtRobotik!";
void IOTAdapter::init()
{
	_txBuf[0] = 0xff;
	for(int i=1; i<15; i++)
		_txBuf[i] = initCode[i-1];
	sendReceive();
}

bool IOTAdapter::enable()
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_ENABLE;
   _txBuf[10] = 0xEE;
   sendReceive();
   return 1;
}

bool IOTAdapter::disable()
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_DISABLE;
   _txBuf[10] = 0xEE;
   std::cout << "Sending disable command" << std::endl;
   sendReceive();
   return 1;
}

bool IOTAdapter::sync()
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_SYNC;
   _txBuf[10] = 0xEE;
   sendReceive();
   return 1;
}

bool IOTAdapter::setRPM(float rpm[8])
{
  int vel[8];
  for(int i=0; i<8; i++)
    vel[i] = (int)(rpm[i]*100.f);
  _txBuf[0] = 0xFF;
  _txBuf[1] = CMD_MOTOR_SETRPM;
  for(int i=0; i<8; i++)
  {
    _txBuf[2*i+3] = (char)(vel[i] >> 8) & 0xFF;
    _txBuf[2*i+2] = (char)(vel[i])      & 0xFF;
  }
  //_txBuf[18] = 0x0;
  sendReceive();
  return 1;
}

void IOTAdapter::sendReceive()
{
    timeval clock;
    double now = 0.0;
    do
    {
       ::gettimeofday(&clock, 0);
       now = static_cast<double>(clock.tv_sec) + static_cast<double>(clock.tv_usec) * 1.0e-6;
    }while((now - _timeCom) < 0.010);
    _timeCom = now;
   _uart->send((char*)_txBuf, 18);
   //usleep(1000);
   return;
   _uart->receive(_rxBuf, 32);

   if(!_rawdata)
   {
      int16_t* ibuf = (int16_t*)(&_rxBuf[9]);
      _q[0] = ((float)ibuf[0])/10000.f;
      _q[1] = ((float)ibuf[1])/10000.f;
      _q[2] = ((float)ibuf[2])/10000.f;
      _q[3] = ((float)ibuf[3])/10000.f;
      
      int16_t temp = _rxBuf[18];
      temp         = temp << 8;
      temp        |= _rxBuf[17];
      _temperature = ((float)temp) / 100.f;
   }
   else
   {
      for(int i=0; i<3; i++)
      {
         int16_t  val     = _rxBuf[10+2*i];
         val              = val << 8;
         val             |= _rxBuf[9+2*i];
         // convert from mg*10 to g
         _acceleration[i] = ((float)val)/10000.f;

         val              = _rxBuf[16+2*i];
         val              = val << 8;
         val             |= _rxBuf[15+2*i];
         // convert from mdps/10 to dps
         _angularRate[i]  = ((float)val)/100.f;
         
         _temperature = -273.f;
      }
   }
   for(int i=0; i<4; i++)
   {
      int16_t val  = _rxBuf[2*i+2] << 8;
      val         |= _rxBuf[2*i+1];
      _rpm[i]      = ((float)val)/100.f;
   
      unsigned short distanceInMM = _rxBuf[22+2*i];
      distanceInMM                = distanceInMM << 8;
      distanceInMM               |= _rxBuf[21+2*i];
      _ranges[i]                  = (float)distanceInMM;
      _ranges[i]                 /= 1000.f;
   }
   unsigned int voltage = _rxBuf[30];
   voltage              = voltage << 8;
   voltage             |= _rxBuf[29];
   _systemVoltage       = (float) voltage / 100.f;
   uint8_t current = _rxBuf[31];
   _loadCurrent = ((float) current)/20.f;
}

} // namespace
