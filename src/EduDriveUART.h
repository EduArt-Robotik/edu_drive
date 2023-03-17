#ifndef __EDU_DRIVE_UART_H
#define __EDU_DRIVE_UART_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <std_srvs/SetBool.h>
#include "IOTAdapter.h"

namespace edu
{

/**
 * @class EduDriveUART
 * @brief Drive interface to EduArt's stackable motor controllers (Version for IOT2050)
 * @author Stefan May
 * @date 17.03.2023
 */
class EduDriveUART
{
public:
    /**
     * @brief Constructor
     *
     */
    EduDriveUART(std::vector<float> kinematics, bool verbosity);

    /**
     * @brief Destroy the Edu Drive object
     *
     */
    ~EduDriveUART();

    /**
     * @brief Blocking ROS handler method. Call this method to enter the ROS message loop.
     *
     */
    void run();

    /**
     * @brief Enable all drives
     * 
     */
    void enable();

    /**
     * @brief Disable all drives
     * 
     */
    void disable();

    /**
     * @brief Method called by ROS as joystick data is available
     * 
     * @param joy joystick data
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd);
    
    void checkLaggyConnection();
    
private:

    void controlMotors(float vFwd, float vLeft, float omega);

    bool enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

    ros::NodeHandle     _nh;
    ros::Subscriber     _subJoy;
    ros::Subscriber     _subVel;
    ros::ServiceServer  _srvEnable;
    ros::Publisher      _pubEnabled;
    ros::Publisher      _pubRPM;
    ros::Publisher      _pubTemp;
    ros::Publisher      _pubVoltageMCU;
    ros::Publisher      _pubCurrentMCU;
    ros::Publisher      _pubVoltageDrive;
    ros::Publisher      _pubCurrentDrive;
    ros::Publisher      _pubIMU;
    ros::Publisher      _pubOrientation;

    ros::Time           _lastCmd;       // Time elapsed since last call

    // Interface to IOT2050
	 edu::IOTAdapter _adapter;

    // Kinematic vector
    std::vector<float> _kinematics;

    double _vMax;
    double _omegaMax;

    bool _enabled;
    bool _verbosity;
};

} // namespace

#endif //__EDU_DRIVE_UART_H
