#ifndef _CARRIERBOARD_H_
#define _CARRIERBOARD_H_

#include "can/SocketCAN.h"

namespace edu
{

/**
 * @class CarrierBoard
 * @brief Interface to EduArt's robot carrier board.
 * @author Stefan May
 * @date 27.04.2022
 */
class CarrierBoard : public SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] params motor parameters
   * @param[in] verbosity verbosity output flag
   */
  CarrierBoard(SocketCAN* can, bool verbosity=false);

  /**
   * Destructor
   */
  ~CarrierBoard();
  
  /**
   * @brief Get orientation as quaternion
   * @param[out] q Orientation quaternion (layout: [w x y z])
   */
  void getOrientation(double q[4]);
  
  /**
   * @brief Get temperature of carrier board
   * @return temperature in degree Celsius
   */
  float getTemperature();

  /**
   * @brief Get voltage of MCU supply
   * @return voltage [V]
   */
  float getVoltageMCU();

  /**
   * @brief Get voltage of power supply of drives
   * @return voltage [V]
   */
  float getVoltageDrive();
  
  /**
   * @brief Get current consumed by MCU
   * @return current [A]
   */
  float getCurrentMCU();
  
  /**
   * @brief Get current consumed by Drive
   * @return current [A]
   */
  float getCurrentDrive();

private:

    void notify(struct can_frame* frame);

    SocketCAN*       _can;

    can_frame        _cf;

    int32_t          _inputAddress;     // Input address (CAN ID) of carrier board

    int32_t          _outputAddress;    // Output address (CAN ID) of carrier board

    int32_t          _broadcastAddress; // Broadcast address for the distribution of CAN data to multiple nodes

    double           _q[4];             // Orientation data as quaternion (layout [w x y z])

    float            _temperature;      // Temperature of surface of carrier board

    float            _voltageMCU;       // Voltage supply for powering MCU

    float            _voltageDrive;     // Voltage supply for powering Drives
    
    float            _currentMCU;       // Current consumed by MCU
    
    float            _currentDrive;     // Current consumed by Drive
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
    bool             _init;
};

} // namespace

#endif // _CARRIERBOARD_H_
