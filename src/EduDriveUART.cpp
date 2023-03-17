#include "EduDriveUART.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include <fcntl.h>

namespace edu
{

    EduDriveUART::EduDriveUART(std::vector<float> kinematics, bool verbosity)
    {
        _verbosity = verbosity;
        _enabled = false;

        _subJoy = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &EduDriveUART::joyCallback, this);
        _subVel = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &EduDriveUART::velocityCallback, this);
        _srvEnable = _nh.advertiseService("enable", &EduDriveUART::enableCallback, this);

        // Publisher of motor shields
        _pubEnabled = _nh.advertise<std_msgs::ByteMultiArray>("enabled", 1);
        _pubRPM = _nh.advertise<std_msgs::Float32MultiArray>("rpm", 1);

        // Publisher of carrier shield
        _pubTemp = _nh.advertise<std_msgs::Float32>("temperature", 1);
        _pubVoltageMCU   = _nh.advertise<std_msgs::Float32>("voltageMCU", 1);
        _pubCurrentMCU   = _nh.advertise<std_msgs::Float32>("currentMCU", 1);
        _pubVoltageDrive = _nh.advertise<std_msgs::Float32>("voltageDrive", 1);
        _pubCurrentDrive = _nh.advertise<std_msgs::Float32>("currentDrive", 1);
        _pubIMU          = _nh.advertise<sensor_msgs::Imu>("imu", 1);
        _pubOrientation  = _nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

        	  
		  // check, if vector is divisible by 3
        bool isKinematicsValid = (kinematics.size()<=24 && (kinematics.size()%3==0));
        if(!isKinematicsValid)
        {
            std::cout << "#EduDrive Kinematic vectors does not fit to drive concept. Vectors of lenght==3 are expected." << std::endl;
            exit(1);
        }
        
        _kinematics = kinematics;
        
        // TODO: calculate _vMax and _omegaMax
        _vMax     = 0.5f;
		  _omegaMax = 1.f;
        ROS_INFO_STREAM("Instanciated robot with vMax: " << _vMax << " m/s and omegaMax: " << _omegaMax << " rad/s");
        
        _adapter.init();
        usleep(10000);
        _adapter.sync();
    }

    EduDriveUART::~EduDriveUART()
    {
        _adapter.disable();
    }

    void EduDriveUART::run()
    {
        _lastCmd = ros::Time::now();

        ros::Timer timerCheckLaggyConnection = _nh.createTimer(ros::Duration(0.5),
                                                               std::bind(&EduDriveUART::checkLaggyConnection, this));

        ros::spin();
    }

    void EduDriveUART::enable()
    {
        ROS_INFO("Enabling robot");

        // TODO: check proper voltage level        
        /*if(voltageDrive < 3.0)
        {
            ROS_WARN_STREAM("Unable to enable motor controllers. Low voltage on motor power supply rail");
            return;
        }*/

        _adapter.enable();
    }

    void EduDriveUART::disable()
    {
        ROS_INFO("Disabling robot");

        _adapter.disable();
    }

    void EduDriveUART::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {
        // Assignment of joystick axes to motor commands
        float fwd = joy->axes[1];                    // Range of values [-1:1]
        float left = joy->axes[0];                   // Range of values [-1:1]
        float turn = joy->axes[2];                   // Range of values [-1:1]
        float throttle = (joy->axes[3] + 1.0) / 2.0; // Range of values [0:1]

        // Enable movement in the direction of the y-axis only when the button 12 is pressed
        if (!joy->buttons[11])
            left = 0;

        static int32_t btn9Prev = joy->buttons[9];
        static int32_t btn10Prev = joy->buttons[10];

        if (joy->buttons[9] && !btn9Prev)
        {
            disable();
        }
        else if (joy->buttons[10] && !btn10Prev)
        {
            enable();
        }

        btn9Prev = joy->buttons[9];
        btn10Prev = joy->buttons[10];

        float vFwd = throttle * fwd * _vMax;
        float vLeft = throttle * left * _vMax;
        float omega = throttle * turn * _omegaMax;

        controlMotors(vFwd, vLeft, omega);
    }

    void EduDriveUART::velocityCallback(const geometry_msgs::Twist::ConstPtr &cmd)
    {
        controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
    }

    bool EduDriveUART::enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
    {
       if(request.data==true)
       {
           ROS_INFO("Enabling robot");
           enable();
       }
       else
       {
           ROS_INFO("Disabling robot");
           disable();
       }
       response.success = true;
       return true;
    }

    void EduDriveUART::controlMotors(float vFwd, float vLeft, float omega)
    {
        _lastCmd = ros::Time::now();

        if (_verbosity)
          std::cout << "#EduDrive Setting RPM to ";
        
        float rpm[8] = {0.f};
        for(int i=0; i<_kinematics.size()/3; i++)
        {
          rpm[i] = _kinematics[i*3] * vFwd + _kinematics[i*3+1] * vLeft + _kinematics[i*3+2] * omega;

          if (_verbosity)
            std::cout << rpm[i] << " ";          
        }
        
        if (_verbosity)
          std::cout << std::endl;

        _adapter.setRPM(rpm);
    }

    void EduDriveUART::checkLaggyConnection()
    {
        ros::Duration dt = ros::Time::now() - _lastCmd;
        bool lag = (dt.toSec() > 0.5);
        if(lag  && _enabled)
        {
            ROS_WARN_STREAM("Lag detected ... deactivate motor control");
            disable();
        }
    }

} // namespace
