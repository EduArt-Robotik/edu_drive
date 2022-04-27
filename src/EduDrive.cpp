#include "EduDrive.h"
#include "std_msgs/Float32MultiArray.h"

EduDrive::EduDrive(ChassisParams &cp, MotorParams &mp, SocketCAN &can)
{
    _subJoy = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &EduDrive::joyCallback, this);

    _pubRPM = _nh.advertise<std_msgs::Float32MultiArray>("rpm", 1);

    _chassisParams = cp;
    _motorParams = mp;
    
    _rpm2ms = cp.wheelDiameter * M_PI / 60.f;

    _vMax = mp.rpmMax * _rpm2ms;

    _omegaMax = 1.0;

    bool verbosity = false;
    _mc.push_back(new MotorControllerCAN(&can, 0, mp, verbosity));
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
    ros::Rate rate(100);
    _lastCmd = ros::Time::now();
    bool run = true;

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Duration dt = ros::Time::now() - _lastCmd;
        bool lag = (dt.toSec() > 0.5);
        if (lag)
        {
            ROS_WARN_STREAM("Lag detected ... deactivate motor control");
        }
        else
        {
        }
        rate.sleep();
    }
}

void EduDrive::enable()
{
    ROS_INFO("Enabling robot");
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        (*it)->enable();
}

void EduDrive::disable()
{
    ROS_INFO("Disabling robot");
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

void EduDrive::controlMotors(float vFwd, float vLeft, float omega)
{
    float w[2];
    w[0] = vFwd / _rpm2ms;
    w[1] = vFwd / _rpm2ms;
    //std::cout << "#w " << w[0] << ", w[1]: " << w[1];
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        (*it)->setRPM(w);
    for (std::vector<MotorControllerCAN *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
    {
        if ((*it)->waitForSync())
        {
            float response[2];
            (*it)->getWheelResponse(response);
            std_msgs::Float32MultiArray msg;
            msg.data.push_back(response[0]);
            msg.data.push_back(response[1]);
            _pubRPM.publish(msg);
            //std::cout << "# RPM[0]: " << response[0] << ", RPM[1]: " << response[1];
            
        }
        else
        {
            std::cout << "# Error synchronizing with device" << (*it)->getCanId() << std::endl;
        };
    }
    //std::cout << std::endl;
}
