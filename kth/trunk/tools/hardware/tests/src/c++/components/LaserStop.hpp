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
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include <Ice/Ice.h>

#include <Robotbase.hpp>

#include <cast/core/CASTComponent.hpp>
#include <OdometryReceiver.hpp>
#include <Scan2dReceiver.hpp>


class LaserStop : public OdometryReceiver,
                  public Scan2dReceiver,
                  virtual public cast::CASTComponent {
public:
  /**
   * Constructor
   */
  LaserStop();

  /**
   * Destructor
   */
  virtual ~LaserStop();

  void receiveOdometry(const Robotbase::Odometry &odom);
  void receiveScan2d(const Laser::Scan2d &scan);

protected:

  virtual void configure(const std::map<std::string,std::string> & config);
  virtual void start();
  virtual void runComponent();

protected:

  std::string m_LaserServerName;
  std::string m_LaserServerHost;
  int m_LaserServerPort;

  std::string m_RobotServerName;
  std::string m_RobotServerHost;
  int m_RobotServerPort;

  bool m_MoveRobot;

  Robotbase::RobotbaseServerPrx m_Robot;

  Robotbase::Pose2d m_InitOdom;
  bool m_Inited;

  Robotbase::Pose2d m_Odom;
  Laser::Scan2d m_Scan;
};
