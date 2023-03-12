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
#if _WITH_MRAA
   _uart = new mraa::Uart(devPath);

   if (_uart->setBaudRate(115200) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setFlowcontrol(false, false) != mraa::SUCCESS) {
      std::cerr << "Error setting flow control UART" << std::endl;
   }
   
   _uart->flush();
#else
   std::cerr << "UART interface not available. MRAA is missing!" << std::endl;
#endif
}
   
IOTAdapter::~IOTAdapter()
{
#if _WITH_MRAA
   delete _uart;
#endif
}

void IOTAdapter::init()
{
#if _WITH_MRAA
	_txBuf[0] = 'E';
	_txBuf[1] = 'd';
	_txBuf[2] = 'u';
	_txBuf[3] = 'A';
	_txBuf[4] = 'r';
	_txBuf[5] = 't';
	_uart->write((char*)_txBuf, 6);
#endif
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
   sendReceive();
   return 1;
}

void IOTAdapter::sendReceive()
{
#if _WITH_MRAA
    timeval clock;
    double now = 0.0;
    do
    {
       ::gettimeofday(&clock, 0);
       now = static_cast<double>(clock.tv_sec) + static_cast<double>(clock.tv_usec) * 1.0e-6;
    }while((now - _timeCom) < 0.008);
    _timeCom = now;
   _uart->write((char*)_txBuf, 11);
   _uart->read(_rxBuf, 32);

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
#else
   std::cerr << "Ignoring UART communication demand." << std::endl;
#endif
}

} // namespace
