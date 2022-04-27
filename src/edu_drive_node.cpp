#include <iostream>

#include "EduDrive.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "edu_drive_node");

    ChassisParams chassisParams;
    MotorParams   motorParams;

    std::string canInterface;
    int maxPulseWidth;
    int frequencyScale;
    int timeout;
    int responseMode;

    // Assign motor channels to motor/wheel mounting
    ros::NodeHandle nh("~");
    nh.param("track",           chassisParams.track,                0.28f);
    nh.param("wheelBase",       chassisParams.wheelBase,            0.0f);
    nh.param("wheelDiameter",   chassisParams.wheelDiameter,        0.06f);
    nh.param("idFrontLeft",     chassisParams.frontLeft.id,         0);
    nh.param("chFrontLeft",     chassisParams.frontLeft.channel,    0);
    nh.param("idFrontRight",    chassisParams.frontRight.id,        0);
    nh.param("chFrontRight",    chassisParams.frontRight.channel,   1);
    nh.param("idCenterLeft",    chassisParams.centerLeft.id,        1);
    nh.param("chCenterLeft",    chassisParams.centerLeft.channel,   0);
    nh.param("idCenterRight",   chassisParams.centerRight.id,       1);
    nh.param("chCenterRight",   chassisParams.centerRight.channel,  1);
    nh.param("idRearLeft",      chassisParams.rearLeft.id,          2);
    nh.param("chRearLeft",      chassisParams.rearLeft.channel,     0);
    nh.param("idRearRight",     chassisParams.rearRight.id,         2);
    nh.param("chRearRight",     chassisParams.rearRight.channel,    1);
    nh.param("direction",       chassisParams.direction,            1);
    nh.param("canInterface",    canInterface, std::string("can0"));
    nh.param("frequencyScale",  frequencyScale,                     32);
    nh.param("inputWeight",     motorParams.inputWeight,            0.8f);
    nh.param("maxPulseWidth",   maxPulseWidth,                      63);
    nh.param("timeout",         timeout, 					              300);
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

    motorParams.responseMode = CAN_RESPONSE_RPM;
    if(responseMode != CAN_RESPONSE_RPM)
       motorParams.responseMode = CAN_RESPONSE_POS;

    SocketCAN can(canInterface);
    can.startListener();
    std::cout << "CAN Interface: " << canInterface << std::endl;

    bool verbosity = false;
    EduDrive drive(chassisParams, motorParams, can, verbosity);
    drive.run();
}
