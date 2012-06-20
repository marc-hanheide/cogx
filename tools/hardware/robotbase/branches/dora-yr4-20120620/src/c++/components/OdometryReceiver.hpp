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

#include <Robotbase.hpp>

#include <cast/core/CASTComponent.hpp>
#include <Ice/Communicator.h>

/**
 * Helper class that makes life a bit easier if you want Odometry pushed
 * to you.
 *
 * @author Patric Jensfelt, Nick Hawes
 * @see
 */
class OdometryReceiver {
public:
  /**
   * Constructor
   */
  OdometryReceiver();

  /**
   * Destructor
   */
  virtual ~OdometryReceiver();

  virtual void receiveOdometry(const Robotbase::Odometry &odom)=0;

  /**
   * Call this method to setup the channel to the server so that data
   * will be pushed. This has to be called when the component is
   * already running, i.e in runComponent in the subclass.
   *
   * Do not change the iceServerPort unless you really know what you do
   *
   * NOTE: Must not be called inside runComponent
   *
   * @param interval desired data interval, <0 --> as fast as possible
   */
  virtual bool setupPushOdometry(cast::CASTComponent &owner,
                                 double interval = -1,
                                 const std::string &serverName = "robot.server");

protected:

  /**
   * This is a helper class used by the OdometryReceiver to get the
   * callbakc when there is new odometry.
   *
   * @author Nick Hawes
   * @see
   */
  class OdometryReceiverHelper : virtual public Robotbase::OdometryPushClient {
    OdometryReceiver * m_receiver;
  public:
    OdometryReceiverHelper(OdometryReceiver * _receiver) :
      m_receiver(_receiver)
    {}
    
    virtual ~OdometryReceiverHelper(){}   

    virtual void receiveOdometry(const Robotbase::Odometry &odom, 
                                 const Ice::Current&) {
      m_receiver->receiveOdometry(odom);
    }
    
  };

};

