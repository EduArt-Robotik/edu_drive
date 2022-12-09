#include "PowerManagement.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <iomanip>
#include <unistd.h>
#include "can/canprotocol.h"

namespace edu
{

PowerManagement::PowerManagement(SocketCAN* can, bool verbosity)
{
  _verbosity = verbosity;
  _voltageSys = 0.f;
  _currentSys = 0.f;
  
  _can       = can;
  makeCanStdID(SYSID_PWRMGMT, 0, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;
  if(verbosity)
    std::cout << "#PowerManagement module CAN Input ID: " << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl;

  canid_t canidOutput = _outputAddress;

  setCANId(canidOutput);
  can->registerObserver(this);
}

PowerManagement::~PowerManagement()
{

}

unsigned short PowerManagement::getCanId()
{
  return _cf.can_id & 0xF;
}

float PowerManagement::getVoltageSys()
{
  return _voltageSys;
}

float PowerManagement::getCurrentSys()
{
  return _currentSys;
}
    
void PowerManagement::notify(struct can_frame* frame)
{
  if(frame->can_dlc==5)
  {
    if(frame->data[0] == CAN_PWR_VOLTAGE)
    {
    	float* val;
    	val = (float*)(&(frame->data[1]));
    	_voltageSys = *val;
    }
    else if(frame->data[0] == CAN_PWR_CURRENT)
    {
      float* val;
    	val = (float*)(&(frame->data[1]));
    	_currentSys = *val;
    }

    if(_verbosity)
	    std::cout << "PowerManagement CANID " << _cf.can_id << " received data (vSys: " << _voltageSys << ", iSys: " << _currentSys << ")" << std::endl;
  }
}

} // namespace
