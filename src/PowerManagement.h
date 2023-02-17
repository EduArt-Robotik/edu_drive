#ifndef _POWERMANAGEMENT_H_
#define _POWERMANAGEMENT_H_

#include "can/SocketCAN.h"
#include <vector>

namespace edu
{

  /**
   * @class PowerManagement
   * @brief CAN interface for EduArt's power management module
   * @author Stefan May
   * @date 17.02.2023
   */
  class PowerManagement : public SocketCANObserver
  {
  public:
    /**
     * Constructor
     * @param[in] can SocketCAN instance
     * @param[in] verbosity verbosity output flag
     */
    PowerManagement(SocketCAN *can, bool verbosity = 0);

    /**
     * Destructor
     */
    ~PowerManagement();

    /**
     * Get assigned canID via constructor
     * @return ID
     */
    unsigned short getCanId();

    /**
     * Accessor to system voltage
     * @return system voltage
     */
    float getVoltageSys();

    /**
     * Accessor to system current
     * @return current through main supply transitor
     */
    float getCurrentSys();

  protected:
  private:
  
    /**
     * Initialize motor controllers (adjust parameters)
     **/
    void init();
    
    bool sendFloat(int cmd, float f);

    /**
     * Implementation of inherited method from SocketCANObserver. This class is getting notified by the SocketCAN,
     * as soon as messages of interest arrive (having the desired CAN ID).
     * @param[in] frame CAN frame
     */
    void notify(struct can_frame *frame);

    SocketCAN *_can;

    int32_t _inputAddress;

    int32_t _outputAddress;

    int32_t _broadcastAddress;

    can_frame _cf;

    bool _verbosity;
   
    float _voltageSys;
    
    float _currentSys;
  };

} // namespace

#endif /* _POWERMANAGEMENT_H_ */
