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
  
  void getOrientation(double q[4]);
  
  float getTemperature();

private:

    void notify(struct can_frame* frame);

    SocketCAN*       _can;

    can_frame        _cf;

    int32_t          _inputAddress;

    int32_t          _outputAddress;

    int32_t          _broadcastAddress;

    double _q[4];  // Orientation data as quaternion (layout [w x y z])

    float _temperature; // Temperature of surface of carrier board
    
    bool _verbosity;
};

} // namespace

#endif // _CARRIERBOARD_H_
