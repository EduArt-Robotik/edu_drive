#include <iostream>

#include "EduDriveUART.h"
#include "IOTAdapter.h"
#include "ros/ros.h"
#include <vector>

//const char* devPath = "/dev/ttyS3";

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "edu_drive_node");

   ros::NodeHandle nh("~eduDrive");

   std::vector<float> kinematics{  100.0,  100.0, 25.0,
                                   100.0,  100.0, 25.0,
                                   100.0,  100.0, 25.0,
                                   100.0,  100.0, 25.0,
                                   100.0,  100.0, 25.0,
                                   100.0,  100.0, 25.0,
                                   0.0,  0.0, 0.0,
                                   0.0,  0.0, 0.0};
   bool verbosity = true;
	edu::EduDriveUART drive(kinematics, verbosity);
	drive.run();

}
