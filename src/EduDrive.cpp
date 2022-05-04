#include "EduDrive.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include <fcntl.h>

namespace edu
{

    EduDrive::EduDrive(std::vector<ControllerParams> cp, SocketCAN &can, bool verbosity)
    {
        _verbosity = verbosity;

        _subJoy = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &EduDrive::joyCallback, this);
        _subVel = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &EduDrive::velocityCallback, this);

        // Publisher of motor shields
        _pubEnabled = _nh.advertise<std_msgs::ByteMultiArray>("enabled", 1);
        _pubRPM = _nh.advertise<std_msgs::Float32MultiArray>("rpm", 1);

        // Publisher of carrier shield
        _pubTemp = _nh.advertise<std_msgs::Float32>("temperature", 1);
        _pubVoltageMCU = _nh.advertise<std_msgs::Float32>("voltageMCU", 1);
        _pubVoltageDrive = _nh.advertise<std_msgs::Float32>("voltageDrive", 1);
        _pubIMU = _nh.advertise<sensor_msgs::Imu>("imu", 1);
        _pubOrientation = _nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

        _vMax = 0.f;

        for (unsigned int i = 0; i < cp.size(); ++i)
        {
            _mc.push_back(new MotorController(&can, cp[i], verbosity));
            
            for(unsigned int j=0; j<_mc[i]->getMotorParams().size(); j++)
            {
            	std::vector<float> kinematics = _mc[i]->getMotorParams()[j].kinematics;
            	if(kinematics.size()==3)
            	{
		         	double kx = kinematics[0];
		         	double kw = kinematics[2];
		         	if(kx>1e-3)
		         	{
		            	float vMax = fabs(cp[i].rpmMax / 60.f * M_PI / kx);
		            	if(vMax > _vMax) _vMax = vMax;
		            	float omegaMax = fabs(cp[i].rpmMax / 60.f * M_PI / kw);
		            	if(omegaMax > _omegaMax) _omegaMax = omegaMax;
		            }
		         }
            }
        }

        ROS_INFO_STREAM("Instanciated robot with vMax: " << _vMax << " m/s and omegaMax: " << _omegaMax << " rad/s");

        _carrier = new CarrierBoard(&can, verbosity);
    }

    EduDrive::~EduDrive()
    {
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            (*it)->disable();
            delete *it;
        }
        delete _carrier;
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

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
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

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
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

    void EduDrive::velocityCallback(const geometry_msgs::Twist::ConstPtr &cmd)
    {
        controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
    }

    void EduDrive::controlMotors(float vFwd, float vLeft, float omega)
    {
        for (unsigned int i = 0; i < _mc.size(); ++i)
        {
            std::vector<float> kinematics0 = _mc[i]->getMotorParams()[0].kinematics;
            std::vector<float> kinematics1 = _mc[i]->getMotorParams()[1].kinematics;
            if(kinematics0.size()!=3 || kinematics1.size()!=3)
            {
                std::cout << "#EduDrive Kinematic vectors does not fit to drive concept. The vector sizes of motorcontroler " << i
                          << " are: " << kinematics0.size() << " and " << kinematics1.size() << ". Vectors of lenght==3 are expected." << std::endl;
                return;
            }
        }
        for (unsigned int i = 0; i < _mc.size(); ++i)
        {
            std::vector<float> kinematics0 = _mc[i]->getMotorParams()[0].kinematics;
            std::vector<float> kinematics1 = _mc[i]->getMotorParams()[1].kinematics;
            float w[2];
            w[0] = kinematics0[0] * vFwd + kinematics0[1] * vLeft + kinematics0[2] * omega;
            w[1] = kinematics1[0] * vFwd + kinematics1[1] * vLeft + kinematics1[2] * omega;

            // Convert from rad/s to rpm
            w[0] *= 60.f / M_PI;
            w[1] *= 60.f / M_PI;
            _mc[i]->setRPM(w);
            if (_verbosity)
                std::cout << "#EduDrive Setting RPM for drive" << i << " to " << w[0] << " " << w[1] << std::endl;
        }
    }

    void EduDrive::receiveCAN()
    {
        std_msgs::Float32MultiArray msgRPM;
        std_msgs::ByteMultiArray msgEnabled;
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            if ((*it)->checkConnectionStatus(200))
            {
                float response[2];
                (*it)->getWheelResponse(response);
                msgRPM.data.push_back(response[0]);
                msgRPM.data.push_back(response[1]);
                msgEnabled.data.push_back((*it)->getEnableState());
            }
            else
            {
                std::cout << "#EduDrive Error synchronizing with device" << (*it)->getCanId() << std::endl;
            };
        }
        _pubRPM.publish(msgRPM);
        _pubEnabled.publish(msgEnabled);

        std_msgs::Float32 msgTemperature;
        msgTemperature.data = _carrier->getTemperature();
        _pubTemp.publish(msgTemperature);

        std_msgs::Float32 msgVoltageMCU;
        msgVoltageMCU.data = _carrier->getVoltageMCU();
        _pubVoltageMCU.publish(msgVoltageMCU);

        std_msgs::Float32 msgVoltageDrive;
        msgVoltageDrive.data = _carrier->getVoltageDrive();
        _pubVoltageDrive.publish(msgVoltageDrive);

        double q[4];
        _carrier->getOrientation(q);
        geometry_msgs::PoseStamped msgOrientation;
        static unsigned int seq = 0;
        msgOrientation.header.seq = seq++;
        msgOrientation.header.stamp = ros::Time::now();
        msgOrientation.header.frame_id = "base_link";
        msgOrientation.pose.position.x = 0;
        msgOrientation.pose.position.y = 0;
        msgOrientation.pose.position.z = 0;
        msgOrientation.pose.orientation.w = q[0];
        msgOrientation.pose.orientation.x = q[1];
        msgOrientation.pose.orientation.y = q[2];
        msgOrientation.pose.orientation.z = q[3];
        _pubOrientation.publish(msgOrientation);
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

} // namespace
