#ifndef __IOTADAPTER_H
#define __IOTADAPTER_H

#include <vector>
#if _WITH_MRAA
#include "mraa/common.hpp"
#include "mraa/uart.hpp"
#else
#include <stdint.h>
#include <string.h>
#endif

namespace edu
{

// Common parameters
#define CMD_ENABLE          0x01
#define CMD_DISABLE         0x02

/**
 * @class IOTAdapter
 * @brief Interface class to IOTAdapter via UART
 * @author Stefan May
 * @date 12.03.2023
 */
class IOTAdapter
{
public:
   /**
    * Default Constructor
    */
   IOTAdapter();

   /**
    * Destructor
    */  
   ~IOTAdapter();

	/**
	 * Initilizing IOTAdapter device, i.e., establishing UART communication
	 */
	void init();

   /**
    * Enable shield (must be done before steering)
    * @return success
    */
   bool enable();
   
   /**
    * Disable shield (no motion can be performed after disabling)
    * @return success
    */
   bool disable();
   
private:

   void sendReceive();

#if _WITH_MRAA
   mraa::Uart* _uart;
#endif

   char _txBuf[11];
   
   char _rxBuf[32];

   float _systemVoltage;
   
   float _loadCurrent;
   
   std::vector<float> _rpm;
      
   std::vector<float> _ranges;
   
   std::vector<float> _acceleration;

   std::vector<float> _angularRate;
   
   std::vector<float> _q;

   double _timeCom;
   
   bool _rawdata;
   
   float _temperature;
     
};

} // namespace

#endif //__IOTADAPTER_H
