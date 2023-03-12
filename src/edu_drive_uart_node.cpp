#include <iostream>

#include "EduDrive.h"
#include "IOTAdapter.h"	
#include "ros/ros.h"
#include <vector>

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "edu_drive_node");

   ros::NodeHandle nh("~eduDrive");

	edu::IOTAdapter adapter;
	adapter.init();
}
