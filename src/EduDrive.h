#ifndef __EDU_DRIVE_H
#define __EDU_DRIVE_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "MotorController.h"
#include "CarrierBoard.h"

namespace edu
{

/**
 * @class EduDrive
 * @brief Drive interface to EduArt's stackable motor controllers
 * @author Stefan May
 * @date 27.04.2022
 */
class EduDrive
{
public:
    /**
     * @brief Constructor
     *
     */
    EduDrive(std::vector<ControllerParams> cp, SocketCAN& can, bool verbosity=false);

    /**
     * @brief Destroy the Edu Drive object
     *
     */
    ~EduDrive();

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

    void receiveCAN();
    
    void checkLaggyConnection();
    
private:

    void controlMotors(float vFwd, float vLeft, float omega);

    ros::NodeHandle     _nh;
    ros::Subscriber     _subJoy;
    ros::Subscriber     _subVel;
    ros::ServiceServer  _srvEnable;
    ros::Publisher      _pubEnabled;
    ros::Publisher      _pubRPM;
    ros::Publisher      _pubTemp;
    ros::Publisher      _pubVoltageMCU;
    ros::Publisher      _pubVoltageDrive;
    ros::Publisher      _pubIMU;
    ros::Publisher      _pubOrientation;

    ros::Time           _lastCmd;       // Time elapsed since last call

    std::vector<MotorController*>  _mc;

    CarrierBoard* _carrier;

    bool _verbosity;
};

} // namespace

#endif //__EDU_DRIVE_H
