#ifndef __EDU_DRIVE_H
#define __EDU_DRIVE_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "MotorController.h"
#include "CarrierBoard.h"

namespace edu
{

struct WheelParams
{
  int id;      // ID of motor controller board, i.e., CAN ID
  int channel; // Channel of motor controller board, i.e., either 0 or 1
};

struct ChassisParams
{
  float track;
  float wheelBase;
  float wheelDiameter;
  WheelParams frontLeft;
  WheelParams frontRight;
  WheelParams centerLeft;
  WheelParams centerRight;
  WheelParams rearLeft;
  WheelParams rearRight;
  int   direction;

  ChassisParams()
  {
    track               = 0.f;
    wheelBase           = 0.f;
    wheelDiameter       = 0.f;
    frontLeft.id        = 0;
    frontLeft.channel   = 0;
    frontRight.id       = 0;
    frontRight.channel  = 0;
    centerLeft.id       = 0;
    centerLeft.channel  = 0;
    centerRight.id      = 0;
    centerRight.channel = 0;
    rearLeft.id         = 0;
    rearLeft.channel    = 0;
    rearRight.id        = 0;
    rearRight.channel   = 0;
    direction           = 0;
  }
};


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
    EduDrive(ChassisParams& cp, MotorParams& mp, SocketCAN& can, bool verbosity=false);

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
    ros::Publisher      _pubVoltage;
    ros::Publisher      _pubIMU;
    ros::Publisher      _pubPose;
    ros::Publisher      _pubTemp;

    ros::Time           _lastCmd;       // Time elapsed since last call

    ChassisParams       _chassisParams;
    MotorParams         _motorParams;
  
    float               _vMax;          // Maximum linear velocity [m/s]
    float               _omegaMax;      // Maximum angular rate [rad/s]
    float               _rpm2ms;        // Conversion factor between [rpm] and [m/s]

    std::vector<MotorController*>  _mc;

    CarrierBoard* _carrier;
};

} // namespace

#endif //__EDU_DRIVE_H
