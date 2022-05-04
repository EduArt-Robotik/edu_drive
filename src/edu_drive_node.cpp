#include <iostream>

#include "EduDrive.h"
#include "ros/ros.h"
#include <vector>

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "edu_drive_node");

   ros::NodeHandle nh("~eduDrive");

   std::vector<edu::ControllerParams> controllerParams;

   // --- System parameters --------
   std::string canInterface;
   int frequencyScale;
   float inputWeight;
   int maxPulseWidth;
   int timeout;

   nh.param("canInterface", canInterface, std::string("can0"));
   nh.param("frequencyScale", frequencyScale, 32);
   nh.param("inputWeight", inputWeight, 0.8f);
   nh.param("maxPulseWidth", maxPulseWidth, 50);
   nh.param("timeout", timeout, 300);

   // Ensure a proper range for the timeout value
   // A lag more than a second should not be tolerated
   if (timeout < 0 && timeout > 1000)
      timeout = 300;

   float kp;
   float ki;
   float kd;
   int antiWindup;
   int invertEnc;
   int responseMode;
   nh.param("kp", kp, 0.0f);
   nh.param("ki", ki, 0.f);
   nh.param("kd", kd, 0.f);
   nh.param("antiWindup", antiWindup, 1);
   nh.param("invertEnc", invertEnc, 0);
   nh.param("responseMode", responseMode, 0);
   // -----------------------------

   // --- Controller parameters ---
   int controllers = 0;
   nh.param("controllers", controllers, 0);

   for(int c=0; c<controllers; c++)
   {
      edu::ControllerParams cp;

      cp.frequencyScale = frequencyScale;
      cp.inputWeight    = inputWeight;
      cp.maxPulseWidth  = maxPulseWidth;
      cp.timeout        = timeout;
      cp.kp             = kp;
      cp.ki             = ki;
      cp.kd             = kd;
      cp.antiWindup     = antiWindup;
      
      std::string controllerID = std::string("controller") + std::to_string(c);
      nh.param(controllerID + "/canID", cp.canID, 0);
      nh.param(controllerID + "/gearRatio", cp.gearRatio, 0.f);
      nh.param(controllerID + "/encoderRatio", cp.encoderRatio, 0.f);
      nh.param(controllerID + "/rpmMax", cp.rpmMax, 0.f);
      nh.param(controllerID + "/invertEnc", cp.invertEnc, invertEnc);

      cp.responseMode   = (responseMode==0 ? edu::CAN_RESPONSE_RPM : edu::CAN_RESPONSE_POS);

      // --- Motor parameters ---------
      for(int d=0; d<2; d++)
      {
         std::string driveID = controllerID + std::string("/drive") + std::to_string(d);
         nh.param(driveID + "/channel", cp.motorParams[d].channel);
         nh.getParam(driveID + "/kinematics", cp.motorParams[d].kinematics);
      }
      // ------------------------------

      controllerParams.push_back(cp);
   }
   // -------------------------

   edu::SocketCAN can(canInterface);
   can.startListener();
   std::cout << "CAN Interface: " << canInterface << std::endl;

   bool verbosity = false;
   edu::EduDrive drive(controllerParams, can, verbosity);
   drive.run();
}
