#include "MotorController.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <iomanip>
#include <unistd.h>
#include "can/canprotocol.h"

namespace edu
{

MotorController::MotorController(SocketCAN* can, unsigned int canID, MotorParams params, bool verbosity)
{
  _verbosity = verbosity;
  
  if(_verbosity)
  {
    std::cout << std::endl << "--- Motor #" << canID << " parameters ---" << std::endl;
    std::cout << "frequencyScale = " << params.frequencyScale << std::endl;
    std::cout << "inputWeight    = " << params.inputWeight << std::endl;
    std::cout << "maxPulseWidth  = " << (int)params.maxPulseWidth << std::endl;
    std::cout << "timeout        = " << params.timeout << std::endl;
    std::cout << "gearRatio      = " << params.gearRatio << std::endl;
    std::cout << "encoderRatio   = " << params.encoderRatio << std::endl;
    std::cout << "rpmMax         = " << params.rpmMax << std::endl;
    std::cout << "responseMode   = " << params.responseMode << std::endl;
    std::cout << "kp             = " << params.kp << std::endl;
    std::cout << "ki             = " << params.ki << std::endl;
    std::cout << "kd             = " << params.kd << std::endl;
    std::cout << "antiWindup     = " << params.antiWindup << std::endl;
    std::cout << "invertEnc      = " << params.invertEnc << std::endl;
    std::cout << "---------------------------" << std::endl << std::endl;
  }

  _params    = params;
  _can       = can;
  makeCanStdID(SYSID_MC2, canID, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;
  std::cout << "# CAN Input ID: " << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl;

  canid_t canidOutput = _outputAddress;

  setCANId(canidOutput);
  can->registerObserver(this);

  _rpm[0] = 0.f;
  _rpm[1] = 0.f;

  _enabled       = false;
  _idSyncSend    = 0;
  _idSyncReceive = 0;

  bool retval = true;
  
  if(!setFrequencyScale(params.frequencyScale))
  {
    std::cout << "# Setting frequency scaling parameter failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!enable())
  {
    std::cout << "# Enabling motor controller failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setInputWeight(params.inputWeight))
  {
    std::cout << "# Setting differential factor of PID controller failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setMaxPulseWidth(params.maxPulseWidth))
  {
    std::cout << "# Setting maximum pulse width failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);

  if(!setTimeout(params.timeout))
  {
    std::cout << "# Setting timeout failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setGearRatio(params.gearRatio))
  {
    std::cout << "# Setting gear ratio failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setEncoderTicksPerRev(params.encoderRatio))
  {
    std::cout << "# Setting encoder parameters failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setKp(params.kp))
  {
    std::cout << "# Setting proportional factor of PID controller failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setKi(params.ki))
  {
    std::cout << "# Setting integration factor of PID controller failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setKd(params.kd))
  {
    std::cout << "# Setting differential factor of PID controller failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!setAntiWindup(params.antiWindup))
  {
    std::cout << "# Setting differential factor of PID controller failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!configureResponse(params.responseMode))
  {
    std::cout << "# Setting response mode failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(1000);
  
  if(!invertEncoderPolarity(params.invertEnc))
  {
    std::cout << "# Setting encoder polarity failed for device " << canID << std::endl;
    retval = false;
  }
  usleep(25000);

  if(!retval)
  {
    std::cout << "# ERROR initializing motor controller with ID " << canID << std::endl;
    std::cout << "-----------------------------------------------";
  }
}

MotorController::~MotorController()
{
  stop();
}

bool MotorController::enable()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_MOTOR_ENABLE;
  return _can->send(&_cf);
}

bool MotorController::disable()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_MOTOR_DISABLE;
  return _can->send(&_cf);
}

bool MotorController::getEnableState()
{
  return _enabled;
}

bool MotorController::broadcastExternalSync()
{
  canid_t idTmp = _cf.can_id;
  _cf.can_id = _broadcastAddress;
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_MOTOR_SYNC;
  bool retval = _can->send(&_cf);
  _cf.can_id = idTmp;
  return retval;
}

bool MotorController::configureResponse(enum CanResponse mode)
{
  _cf.can_dlc = 1;
  if(mode==CAN_RESPONSE_RPM)
    _cf.data[0] = CMD_MOTOR_SENDRPM;
  else
    _cf.data[0] = CMD_MOTOR_SENDPOS;
  return _can->send(&_cf);
}

bool MotorController::invertEncoderPolarity(bool invert)
{
  _cf.can_dlc = 2;
  _cf.data[0] = CMD_MOTOR_INVERTENC;
  if(invert)
    _cf.data[1] = 1;
  else
    _cf.data[1] = 0;
  return _can->send(&_cf);
}

unsigned short MotorController::getCanId()
{
  return _cf.can_id & 0xF;
}

bool MotorController::setTimeout(unsigned short timeoutInMillis)
{
  _cf.can_dlc = 3;
  _cf.data[0] = CMD_MOTOR_SETTIMEOUT;
  _cf.data[1] = (timeoutInMillis >> 8) & 0xFF;
  _cf.data[2] = timeoutInMillis & 0xFF;
  bool retval = _can->send(&_cf);
  if(retval)
    _params.timeout = timeoutInMillis;
  return retval;
}

unsigned short MotorController::getTimeout()
{
  return _params.timeout;
}

bool MotorController::setGearRatio(float gearRatio)
{
  bool retval = sendFloat(CMD_MOTOR_GEARRATIO, gearRatio);
  retval &= sendFloat(CMD_MOTOR_GEARRATIO2, gearRatio);
  if(retval)
    _params.gearRatio = gearRatio;
  return retval;
}

float MotorController::getGearRatio()
{
  return _params.gearRatio;
}

bool MotorController::setEncoderTicksPerRev(float encoderTicksPerRev)
{
  bool retval = sendFloat(CMD_MOTOR_TICKSPERREV, encoderTicksPerRev);
  retval &= sendFloat(CMD_MOTOR_TICKSPERREV2, encoderTicksPerRev);
  if(retval)
    _params.encoderRatio = encoderTicksPerRev;
  return retval;
}

bool MotorController::setFrequencyScale(unsigned short scale)
{
  bool retval = false;

  if(scale>0 && scale<=100)
  {
    _cf.can_dlc = 3;
    _cf.data[0] = CMD_MOTOR_FREQ_SCALE;
    _cf.data[1] = (scale >> 8) & 0xFF;
    _cf.data[2] = scale & 0xFF;
    retval = _can->send(&_cf);
  }
  if(retval)
    _params.frequencyScale = scale;
  return retval;
}

unsigned short MotorController::getFrequencyScale()
{
  return _params.frequencyScale;
}

bool MotorController::setMaxPulseWidth(unsigned char pulse)
{
  bool retval = false;

  if(pulse<=127)
  {
    _cf.can_dlc = 2;
    _cf.data[0] = CMD_MOTOR_SETPWMMAX;
    _cf.data[1] = pulse;
    retval = _can->send(&_cf);
  }
  if(retval)
    _params.maxPulseWidth = pulse;
  return retval;
}

bool MotorController::setPWM(int pwm[2])
{
  _cf.can_dlc = 3;

  int vel1 = (((int)pwm[0]) * 127) / 100;
  int vel2 = (((int)pwm[1]) * 127) / 100;
  _cf.data[0] = CMD_MOTOR_SETPWM;
  _cf.data[1] = (char)(vel1 + 0x7F);
  _cf.data[2] = (char)(vel2 + 0x7F);

  return _can->send(&_cf);
}

bool MotorController::setRPM(float rpm[2])
{
  _cf.can_dlc = 5;

  int vel1 = (int)(rpm[0]*10.f);
  int vel2 = (int)(rpm[1]*10.f);
  _cf.data[0] = CMD_MOTOR_SETRPM;
  _cf.data[1] = (char)(vel1 >> 8) & 0xFF;
  _cf.data[2] = (char)(vel1)      & 0xFF;
  _cf.data[3] = (char)(vel2 >> 8) & 0xFF;
  _cf.data[4] = (char)(vel2)      & 0xFF;

  return _can->send(&_cf);
}

void MotorController::getWheelResponse(float response[2])
{
  if(_params.responseMode == CAN_RESPONSE_RPM)
  {
    response[0] = _rpm[0];
    response[1] = _rpm[1];
  }
  else
  {
    response[0] = _pos[0];
    response[1] = _pos[1];
  }
}

bool MotorController::setKp(float kp)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KP, kp);
  if(retval)
    _params.kp = kp;
  return retval;
}

float MotorController::getKp()
{
  return _params.kp;
}

bool MotorController::setKi(float ki)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KI, ki);
  if(retval)
    _params.ki = ki;
  return retval;
}

float MotorController::getKi()
{
  return _params.ki;
}

bool MotorController::setKd(float kd)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KD, kd);
  if(retval)
    _params.kd = kd;
  return retval;
}

float MotorController::getKd()
{
  return _params.kd;
}

bool MotorController::setAntiWindup(bool activate)
{
   bool retval = false;
  _cf.can_dlc = 2;
  _cf.data[0] = CMD_MOTOR_CTL_ANTIWINDUP;
  _cf.data[1] = activate;
  retval = _can->send(&_cf);
  if(retval)
    _params.antiWindup = activate;
  return retval;
}

bool MotorController::getAntiWindup()
{
  return _params.antiWindup;
}

bool MotorController::setInputWeight(float weight)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_INPUTFILTER, weight);
  if(retval)
    _params.inputWeight = weight;
  return retval;
}

float MotorController::getInputWeight()
{
  return _params.inputWeight;
}

void MotorController::notify(struct can_frame* frame)
{
  if(frame->can_dlc==6)
  {
    if(frame->data[0] == RESPONSE_MOTOR_RPM)
    {
    
      short val1 = (frame->data[1] | (frame->data[2] << 8));
      short val2 = (frame->data[3] | (frame->data[4] << 8));
      _rpm[0] = ((float)val1)/10.f;
      _rpm[1] = ((float)val2)/10.f;
      _pos[0] = 0.f;
      _pos[1] = 0.f;
    }
    else if(frame->data[0] == RESPONSE_MOTOR_POS)
    {
      _rpm[0] = 0.f;
      _rpm[1] = 0.f;
      _pos[0] = (frame->data[1] | (frame->data[2] << 8));
      _pos[1] = (frame->data[3] | (frame->data[4] << 8));
    }
    _enabled = (frame->data[5] != 0);
    _cntReceived++;
    if(_verbosity)
	    std::cout << "CAN #" << _cf.can_id << ": Received (" << _cntReceived << ")" << std::endl;
  }
}

bool MotorController::waitForSync(unsigned int timeoutInMillis)
{
  unsigned int cnt=0;
  unsigned int cntAtStart = _cntReceived;
  while((cntAtStart == _cntReceived) && ((cnt++)<timeoutInMillis || timeoutInMillis==0))
  {
    usleep(1000);
  }
  return (_cntReceived>cntAtStart);
}

void MotorController::stop()
{
  _cf.can_dlc = 3;
  _cf.data[0] = CMD_MOTOR_SETPWM;
  _cf.data[1] = 0x0;
  _cf.data[2] = 0x0;
  _can->send(&_cf);
}

bool MotorController::sendFloat(int cmd, float f)
{
  _cf.can_dlc = 5;

  _cf.data[0] = cmd;
  int* ival = (int*)&f;
  _cf.data[1] = (*ival & 0xFF000000) >> 24;
  _cf.data[2] = (*ival & 0x00FF0000) >> 16;
  _cf.data[3] = (*ival & 0x0000FF00) >> 8;
  _cf.data[4] = (*ival & 0x000000FF);

  return _can->send(&_cf);
}

} // namespace