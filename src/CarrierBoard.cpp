#include "CarrierBoard.h"
#include <iostream>
#include "can/canprotocol.h"

namespace edu
{

CarrierBoard::CarrierBoard(SocketCAN* can, bool verbosity)
{
  _verbosity = verbosity;
  
  _q[0] = 1.0;
  _q[1] = 0.0;
  _q[2] = 0.0;
  _q[3] = 0.0;

  _temperature = -273.f;

  makeCanStdID(SYSID_RPI_ADAPTER, RPI_ADAPTER, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;
  std::cout << "# CAN Input ID: " << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl;

  canid_t canidOutput = _outputAddress;

  setCANId(canidOutput);
  can->registerObserver(this);
}

CarrierBoard::~CarrierBoard()
{

}

void CarrierBoard::getOrientation(double q[4])
{
  q[0] = _q[0];
  q[1] = _q[1];
  q[2] = _q[2];
  q[3] = _q[3];
}

float CarrierBoard::getTemperature()
{
  return _temperature;
}

void CarrierBoard::notify(struct can_frame* frame)
{
  if(frame->can_dlc==8)
  {
    int16_t* data = (int16_t*)frame->data;
    _q[0] = ((double)data[0]) / 10000.0;
    _q[1] = ((double)data[1]) / 10000.0;
    _q[2] = ((double)data[2]) / 10000.0;
    _q[3] = ((double)data[3]) / 10000.0;
    
    if(_verbosity)
      std::cout << "w=" << _q[0] << " x=" << _q[1] << " y=" << _q[2] << " z=" << _q[3] << std::endl;
  }
  else if(frame->can_dlc==2)
  {
    int16_t* temperature = (int16_t*)frame->data;
    _temperature = ((float) *temperature) / 100.f;
  }
}

} // namespace
