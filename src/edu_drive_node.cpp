#include <iostream>

#include "EduDrive.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "edu_drive_node");

    std::vector<edu::DriveKinematics> kinematics;

    std::string canInterface;
    int maxPulseWidth;
    int frequencyScale;
    int timeout;
    int responseMode;

    // Assign motor channels to motor/wheel mounting
    ros::NodeHandle nh("~");
    
    nh.param("canInterface",    canInterface,                       std::string("can0"));
    nh.param("frequencyScale",  frequencyScale,                     32);
    
    edu::MotorParams motorParams;
    nh.param("inputWeight",     motorParams.inputWeight,            0.8f);
    nh.param("maxPulseWidth",   maxPulseWidth,                      63);
    nh.param("timeout",         timeout,                            300);
    nh.param("gearRatio",       motorParams.gearRatio,              70.f);
    nh.param("encoderRatio",    motorParams.encoderRatio,           64.f);
    nh.param("rpmMax",          motorParams.rpmMax,                 140.f);
    nh.param("responseMode",    responseMode,                       0);
    nh.param("kp",              motorParams.kp,                     0.5f);
    nh.param("ki",              motorParams.ki,                     300.f);
    nh.param("kd",              motorParams.kd,                     0.f);
    nh.param("antiWindup",      motorParams.antiWindup,             1);
    nh.param("invertEnc",       motorParams.invertEnc,              1);

    // Ensure a proper range for the timeout value
    // A lag more than a second should not be tolerated
	 if(timeout<0 && timeout>1000)
	    timeout = 300;
	 motorParams.timeout        = timeout;
	 motorParams.frequencyScale = frequencyScale;
	 motorParams.maxPulseWidth  = maxPulseWidth;

    motorParams.responseMode = edu::CAN_RESPONSE_RPM;
    if(responseMode != edu::CAN_RESPONSE_RPM)
       motorParams.responseMode = edu::CAN_RESPONSE_POS;
       
    // Check, if motor assignment is plausible
    int drives = 0;
    nh.param("drives", drives, 0);
    for(unsigned int i=0; i<drives; ++i)
    {
    	std::string driveID = std::string("drive") + std::to_string(i);
    	std::vector<float> driveParams;
    	nh.getParam(driveID, driveParams);
    	if(driveParams.size()!=5)
    	{
    	  std::cout << "Warning: Launch file parameters are not plausible for " << driveID << std::endl;
    	  exit(1);
    	}
    	
    	edu::DriveKinematics k;
    	k.canID   = driveParams[0];
      k.channel = driveParams[1];
      k.kX      = driveParams[2];
      k.kY      = driveParams[3];
      k.kOmega  = driveParams[4];
      k.mp      = motorParams;
      if(i==1) k.mp.invertEnc = 0;
      kinematics.push_back(k);
    }
    
    edu::SocketCAN can(canInterface);
    can.startListener();
    std::cout << "CAN Interface: " << canInterface << std::endl;

    bool verbosity = true;
    edu::EduDrive drive(kinematics, can, verbosity);
    drive.run();
}
