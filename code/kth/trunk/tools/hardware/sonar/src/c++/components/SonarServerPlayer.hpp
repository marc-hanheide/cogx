#ifndef SonarServerPlayer_hpp
#define SonarServerPlayer_hpp

#include <Ice/Ice.h>

#include <Sonar.hpp>

#include <cast/core/CASTComponent.hpp>
#include <cast/core/CASTTimer.hpp>

#include <libplayerc++/playerc++.h>

namespace Sonar {

/**
 * A server that implements the Sonar::SonarServer interface and
 * connects to the hardware via player.
 * 
 * @param --player-host host where player is running
 * @param --player-port port for player server
 * @param --server-name name to give this server
 * @param --min-push-interval minimum time [s] between two pushes of data to client
 * @param --time-offset the time [s] to subtract the current time to get the time 
 * 
 */
class SonarServerPlayer : public Sonar::SonarServer,
                          public cast::CASTComponent {
public:

  /**
   * Constructor
   */
  SonarServerPlayer();

  /**
   * Destructor
   */
  ~SonarServerPlayer();

  Sonar::SonarScan2d pullSonarScan2d(const Ice::Current & _crt);
  
//   void registerSonarScan2dPushClient(const Sonar::SonarScan2dPushClientPrx &, 
// 				     const Ice::Current&);

protected:
  virtual void configure(const std::map<std::string,std::string> & config);
  
  virtual void start();
  
  virtual void runComponent();
  
  virtual void stop();
  
protected:
  
//   class SonarScan2dClient {
//   public:
//     Sonar::SonarScan2dPushClientPrx prx;
//     double interval;
//     cast::CASTTimer timer;
//   };
//   std::vector<SonarScan2dClient> m_PushClients;
 
  std::string m_PlayerHost;
  int m_PlayerPort;
  int m_PlayerPosDeviceId;

 
//   /// The minimum time between two pushes [s], can be used to limit
//   /// use of bandwidth. If <0 no limit is used
//   double m_MinPushInterval;

  PlayerCc::PlayerClient *m_PlayerClient;
  PlayerCc::SonarProxy *m_Sonar;
  Sonar::SonarScan2d m_Scan;

  cast::cdl::CASTTime m_TimeOffset;
};

}; // namespace Sonar

#endif // SonarServerPlayer_hpp
