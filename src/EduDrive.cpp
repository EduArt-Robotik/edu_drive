#include "EduDrive.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include <fcntl.h>

EduDrive::EduDrive(ChassisParams &cp, MotorParams &mp, SocketCAN &can, bool verbosity)
{
    _subJoy = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &EduDrive::joyCallback, this);
    _subVel = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &EduDrive::velocityCallback, this);
  
    _pubEnabled = _nh.advertise<std_msgs::ByteMultiArray>("enabled", 1);
    _pubRPM     = _nh.advertise<std_msgs::Float32MultiArray>("rpm", 1);
    _pubVoltage = _nh.advertise<std_msgs::Float32>("voltage", 1);
    _pubTemp    = _nh.advertise<std_msgs::Float32>("temperature", 1);
    _pubIMU     = _nh.advertise<sensor_msgs::Imu>("imu", 1);
    _pubPose    = _nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    _chassisParams = cp;
    _motorParams = mp;
    
    _rpm2ms = cp.wheelDiameter * M_PI / 60.f;

    _vMax = mp.rpmMax * _rpm2ms;

    _omegaMax = 1.0;

    _mc.push_back(new MotorControllerCAN(&can, 0, mp, verbosity));
    _mc.push_back(new MotorControllerCAN(&can, 1, mp, verbosity));
}

EduDrive::~EduDrive()
{
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
    {
        (*it)->disable();
        delete *it;
    }
}

void EduDrive::run()
{
    _lastCmd = ros::Time::now();

    ros::Timer timerReceiveCAN = _nh.createTimer(ros::Duration(0.1),
                                                 std::bind(&EduDrive::receiveCAN, this));
                                                 
    ros::Timer timerCheckLaggyConnection = _nh.createTimer(ros::Duration(0.5),
                                                 std::bind(&EduDrive::checkLaggyConnection, this));
                                                 
    ros::spin();
}

void EduDrive::enable()
{
    ROS_INFO("Enabling robot");
    
    // This is added for the RPi version using GPIO16 to enable all motor controllers
    int fd = open("/sys/class/gpio/gpio16/value", O_WRONLY);
    if (fd == -1)
    {
        ROS_WARN_STREAM("Unable to enable motor controllers. It the GPIO16 pin configured as output?");
        return;
    }
    write(fd, "1", 1);
	 close(fd);
	    
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        (*it)->enable();
}

void EduDrive::disable()
{
    ROS_INFO("Disabling robot");
    
    // This is added for the RPi version using GPIO16 to enable all motor controllers
    int fd = open("/sys/class/gpio/gpio16/value", O_WRONLY);
    if (fd == -1)
    {
        ROS_WARN_STREAM("Unable to disable motor controllers. It the GPIO16 pin configured as output?");
        // At this point we go on to send at least the disable command via CAN.
    }
    else
    {
    	write(fd, "0", 1);
		 close(fd);
    }
	 
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        (*it)->disable();
}

void EduDrive::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
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
    
    _lastCmd = ros::Time::now();
}

void EduDrive::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

void EduDrive::controlMotors(float vFwd, float vLeft, float omega)
{
    float w[2];
    w[0] = vFwd / _rpm2ms;
    w[1] = vFwd / _rpm2ms;
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        (*it)->setRPM(w);
}

void EduDrive::receiveCAN()
{
    std_msgs::Float32MultiArray msgRPM;
    std_msgs::ByteMultiArray msgEnabled;
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
    {
        if ((*it)->waitForSync())
        {
            float response[2];
            (*it)->getWheelResponse(response);
            msgRPM.data.push_back(response[0]);
            msgRPM.data.push_back(response[1]);
            msgEnabled.data.push_back((*it)->getEnableState());
        }
        else
        {
            std::cout << "# Error synchronizing with device" << (*it)->getCanId() << std::endl;
        };
    }
    _pubRPM.publish(msgRPM);       
    _pubEnabled.publish(msgEnabled);
}

void EduDrive::checkLaggyConnection()
{
    ros::Duration dt = ros::Time::now() - _lastCmd;
    bool lag = (dt.toSec() > 0.5);
    if (lag)
    {
        ROS_WARN_STREAM("Lag detected ... deactivate motor control");
        disable();
    }
}
