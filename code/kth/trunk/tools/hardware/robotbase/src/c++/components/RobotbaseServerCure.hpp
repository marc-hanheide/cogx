//
// = FILENAME
//    RobotbaseServerCure.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef RobotbaseServerCure_hpp
#define RobotbaseServerCure_hpp

#include <Ice/Ice.h>

#include "JoystickDrivable.hpp"
#include <Robotbase.hpp>

#include <cast/core/CASTComponent.hpp>
#include <cast/core/CASTTimer.hpp>

#include <Hardware/Robot/P2OS/P2OSDeviceAddress.hh>

namespace Robotbase {

/**
 * Component that implements the Robotbase::RobotbaseServer interface
 * and talks to the hardware via player.
 *
 * @param --server-name <name to give this server>
 *@param  --min-push-interval <minimum time [s] between two pushes of data to client>
 * @param --cure-config-file <file with information about the connection to the robot>
 *
 * @author Patric Jensfelt
 * @see
 */
class RobotbaseServerCure : public Robotbase::RobotbaseServer,
                            public JoystickDrivable,
                            public cast::CASTComponent {
public:
  /**
   * Constructor
   */
  RobotbaseServerCure();

  /**
   * Destructor
   */
  ~RobotbaseServerCure();

  Robotbase::Odometry pullOdometry(const Ice::Current&);

  void execMotionCommand(const ::Robotbase::MotionCommand& cmd,
                         const Ice::Current&);
  
  void registerOdometryPushClient(const Robotbase::OdometryPushClientPrx&, 
                                  Ice::Double interval,  
                                  const Ice::Current&);
protected:
  virtual void configure(const std::map<std::string,std::string> & config);

  virtual void start();

  virtual void runComponent();

  virtual void stop();

protected:

  class OdometryClient {
  public:
    Robotbase::OdometryPushClientPrx prx;
    double interval;
    cast::CASTTimer timer;
  };
  std::vector<OdometryClient> m_PushClients;

  std::string m_CfgFile;
  Cure::Hal::P2OSSerialInterface *m_P2OSDevice;

  // The name of the Ice server
  std::string m_IceServerName;

  /// The minimum time between two pushes [s], can be used to limit
  /// use of bandwidth. If <0 no limit is used
  double m_MinPushInterval;  

  Robotbase::Odometry m_Odom;
  bool m_MotionCmdGiven;
  Robotbase::MotionCommand m_MotionCmd;
};

}; // namespace Robotbase

#endif // RobotbaseServerCure_hpp
