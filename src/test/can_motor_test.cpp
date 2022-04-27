/**
 * @author Stefan May
 * @date 23.04.2022
 * @brief Test program for CAN interface
 */
#include "MotorControllerCAN.h"
#include <unistd.h>
#include <iostream>
#include <cmath>

using namespace std;

#define _INSTANCES 1
int main(int argc, char* argv[])
{
  MotorParams motorParams;
  motorParams.frequencyScale = 64;    // PWM frequency: 1/frequencyScale x 500kHz
  motorParams.inputWeight    = 0.8f;  // Smoothing parameter for input values: smoothVal = inputWeight x prevVal + (1.f-inputWeight) x newVal
  motorParams.maxPulseWidth  = 100;   // Set maxPulse to apply full power
  motorParams.timeout        = 300;
  motorParams.gearRatio      = 70.f;
  motorParams.encoderRatio   = 64.f;
  motorParams.rpmMax         = 100;
  motorParams.responseMode   = CAN_RESPONSE_RPM;
  motorParams.kp             = 1.0f;
  motorParams.ki             = 100.5f;
  motorParams.kd             = 0.f;
  motorParams.antiWindup     = 1;
  motorParams.invertEnc      = 1;

  SocketCAN can(std::string("can0"));
  can.startListener();

  std::vector<MotorControllerCAN*> mc;
  unsigned int dev = 0;
  for(dev=0; dev<_INSTANCES; dev++)
  {
    MotorControllerCAN* m = new MotorControllerCAN(&can, dev, motorParams, 1);
    mc.push_back(m);
  }

  float w[2] = {0.f, 0.f};
  for(int i=0; i<200; i++)
  {
    if(i%20==0)
      mc[0]->broadcastExternalSync();

    float phase = ((float)i) * (2.f*M_PI) * 0.005;
    float amplitude = 50.f;
    float val = (sin(phase) * amplitude);

    for(dev=0; dev<mc.size(); dev++)
    {
      w[0] = val;
      mc[dev]->setRPM(w);
	 }
    std::cout << "setRPM: " << w[0] << " " << w[1] << std::endl;
    for(dev=0; dev<mc.size(); dev++)
    {
      if(mc[dev]->waitForSync())
      {
        float response[2];
        mc[dev]->getWheelResponse(response);
        std::cout << " " << response[0] << " " << response[1];
      }
      else
      {
        std::cout << "# Error synchronizing with device" << mc[dev]->getCanId() << std::endl;
      };
    }
    std::cout << std::endl;
    usleep(10000);
  }

  for(dev=0; dev<mc.size(); dev++)
  {
    delete mc[dev];
  }
}
