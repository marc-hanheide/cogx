//
// = FILENAME
//    
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt, Nick Hawes
//
/*----------------------------------------------------------------------*/

#include <Ice/Ice.h>

#include <Laser.hpp>

#include <cast/core/CASTComponent.hpp>



/**
 * Helper class that makes life a bit easier if you want Scan2d pushed
 * to you.
 *
 * @author Patric Jensfelt, Nick Hawes
 * @see
 */
class Scan2dReceiver {
public:
  /**
   * Constructor
   */
  Scan2dReceiver();

  /**
   * Destructor
   */
  virtual ~Scan2dReceiver();


  virtual void receiveScan2d(const Laser::Scan2d &scan)=0;

  /**
   * Call this method to setup the channel to the server so that data
   * will be pushed. This has to be called when the component is
   * already running, i.e in runComponent in the subclass.
   *
   * Do not change the iceServerPort unless you really know what you do
   *
   * @param interval desired data interval, <0 --> as fast as possible
   */
  virtual bool setupPushScan2d(cast::CASTComponent &owner, 
                               double interval = -1,
                               const std::string &serverName="laser.server");

protected:

  /**
   * Helper class that let the Scan2dReceiver register its callback function
   *
   * @author Nick Hawes
   * @see
   */
  class Scan2dReceiverHelper : virtual public Laser::Scan2dPushClient {
    Scan2dReceiver * m_receiver;
  public:
    Scan2dReceiverHelper(Scan2dReceiver * _receiver) :
      m_receiver(_receiver)
    {}
    
    virtual void receiveScan2d(const Laser::Scan2d &scan, const Ice::Current&) {
      m_receiver->receiveScan2d(scan);
    }
    
  };

};
