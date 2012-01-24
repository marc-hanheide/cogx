//
// = FILENAME
//    LaserServerPlayer.hpp
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

#ifndef LaserServerPlayer_hpp
#define LaserServerPlayer_hpp

#include <Ice/Ice.h>

#include <Laser.hpp>

#include <cast/core/CASTComponent.hpp>
#include <cast/core/CASTTimer.hpp>

#include <libplayerc++/playerc++.h>

namespace Laser {

/**
 * A server that implements the Laser::LaserServer interface and
 * connects to the hardware via player.
 * 
 * @param --player-host host where player is running
 * @param --player-port port for player server
 * @param --server-name name to give this server
 * @param --min-push-interval minimum time [s] between two pushes of data to client
 * @param --time-offset the time [s] to subtract the current time to get the time 
 *   when the scan was acquired. This is used to tune the timestamping
 *   so that it gets closer to the true acquisition time. With this variable
 *   unspecified or set to 0, the timestamp will be the time when the Read()
 *   function in player returns. Specifying 0.1 will subtract 0.1s from that
 *   time and thus say that scan was acquired 0.1s before Read() returned.
 * @param --rand-data if specified the server will not connect to player but instead
 *   generate random data. Good sometimes for debugging without stage or 
 *   hardware
 * @param --prob-fake-min-outlier number 0-1 that gives the probability of faking an
 *   outlier inside minRange for an indivual reading. Only use this if you 
 *   know what you are doing and typically only in simulation
 * @param --prob-fake-max-outlier number 0-1 that gives the probability of faking an
 *   outlier at max range for an indivual reading. Only use this if you know 
 *   what you are doing and only in simulation
 *
 * @author Patric Jensfelt
 * @see
 */
class LaserServerPlayer : public cast::CASTComponent {
protected:

	class LaserServerI : public Laser::LaserServer,
				public cast::CASTComponent {
		private:
			LaserServerPlayer *svr;

		public:
		LaserServerI(LaserServerPlayer *_svr) : svr(_svr) {}

		Laser::Scan2d pullScan2d(const Ice::Current&);
		void registerScan2dPushClient(const Laser::Scan2dPushClientPrx&, 
                                Ice::Double interval,  
                                const Ice::Current&);
	};
  
public:
  /**
   * Constructor
   */
  LaserServerPlayer();

  /**
   * Destructor
   */
  ~LaserServerPlayer();

protected:
  virtual void configure(const std::map<std::string,std::string> & config);

  virtual void start();

  virtual void runComponent();

  virtual void stop();

  void saveScanToFile(Laser::Scan2d scan);

protected:

  class Scan2dClient {
  public:
    Laser::Scan2dPushClientPrx prx;
    double interval;
    cast::CASTTimer timer;
  };
  std::vector<Scan2dClient> m_PushClients;

  bool m_RandData;

  std::string m_PlayerHost;
  int m_PlayerPort;
  int m_PlayerPosDeviceId;

  // The name of the Ice server
  std::string m_IceServerName;

  /// True if we want to use the Ranger interface which we should if
  /// we are using a hokuyu with the aist interface and not a Sick
  /// sensor
  bool m_UseRangerInterface;

  /// The minimum time between two pushes [s], can be used to limit
  /// use of bandwidth. If <0 no limit is used
  double m_MinPushInterval;

  /// Generate random (fake) < minRange outliers with this probability
  /// prob<0 --> do not do it at all
  double m_ProbFakeMinOutlier;

  /// Generate random (fake) max range outliers with thsi probability
  /// prob<0 do not do it at all
  double m_ProbFakeMaxOutlier;

  PlayerCc::PlayerClient *m_PlayerClient;
  PlayerCc::LaserProxy *m_Laser;
  Laser::Scan2d m_Scan;

  cast::cdl::CASTTime m_TimeOffset;

  /// If this variable is > 0 it means that we want to force a certain
  /// discretization on the range data. This can be used to simulate
  /// for example a PLS sensor with 5cm discretization on the range
  /// readings. The variable should be expressed in [m]
  /// r_new = m_Discr * int(r_old / m_Discr + 0.5)
  double m_Discr;

  bool m_saveToFile;
  std::string m_saveDirectory;
};

}; // namespace Laser

#endif // LaserServerPlayer_hpp
