//
// = FILENAME
//    RobotbaseServerPlayer.hpp
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

#ifndef RobotbaseServerPlayer_hpp
#define RobotbaseServerPlayer_hpp

#include <Ice/Ice.h>

#include <Robotbase.hpp>

#include <CASTComponent.hpp>
#include <CASTTimer.hpp>
#include "JoystickDrivable.hpp"

#include <libplayerc++/playerc++.h>

// Cure
#include <Utils/TimestampFilter.hh>

namespace Robotbase {

/**
 * Component that implements the Robotbase::RobotbaseServer interface
 * and talks to the hardware via player.
 *
 * @param --player-host host where player is running (default localhost)
 * @param --player-port port for player server (default 6665)
 * @param --server-name name to give this server (default RobotbaseServer)
 * @param --min-push-interval minimum time [s] between two pushes of data to client
 * @param --time-offset the time [s] to subtract the current time to get the time 
 *   when the odometry was sampled. This is used to tune the timestamping
 *   so that it gets closer to the true acquisition time. With this variable
 *   unspecified or set to 0, the timestamp will be the time when the Read()
 *   function in player returns. Specifying 0.1 will subtract 0.1s from that
 *   time and thus say that odometry was sampled 0.1s before Read() returned.
 * @param --rand-data if specified the server will not connect to player but instead
 *   generate random data. Good sometimes for debugging without stage or 
 *   hardware
 *
 * @author Patric Jensfelt
 * @see
 */
class RobotbaseServerPlayer : public Robotbase::RobotbaseServer,
                              public JoystickDrivable,
                              public cast::CASTComponent {
public:
  /**
   * Constructor
   */
  RobotbaseServerPlayer();

  /**
   * Destructor
   */
  ~RobotbaseServerPlayer();

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

  bool m_RandData;

  std::string m_PlayerHost;
  int m_PlayerPort;
  int m_PlayerPosDeviceId;

  bool m_EnableMotors;

  // The name of the Ice server
  std::string m_IceServerName;

  /// The minimum time between two pushes [s], can be used to limit
  /// use of bandwidth. If <0 no limit is used
  double m_MinPushInterval;  

  PlayerCc::PlayerClient *m_PlayerClient;
  PlayerCc::Position2dProxy *m_Position;
  Robotbase::Odometry m_Odom;

  Cure::TimestampFilter m_TimeFilter;
};

}; // namespace Robotbase;

#endif // RobotbaseServerPlayer_hpp
