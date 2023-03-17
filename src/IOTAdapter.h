#ifndef __IOTADAPTER_H
#define __IOTADAPTER_H

#include <vector>
#include <stdint.h>
#include <string.h>
#include "uart/SerialPort.h"

namespace edu
{

// Common parameters
#define CMD_ENABLE          0x01
#define CMD_DISABLE         0x02
#define CMD_MOTOR_SETRPM    0x11
#define CMD_SYNC            0x13

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
   
   bool sync();
   
   bool setRPM(float rpm[8]);
   
private:

   void sendReceive();

   SerialPort* _uart;

   char _txBuf[18];
   
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
