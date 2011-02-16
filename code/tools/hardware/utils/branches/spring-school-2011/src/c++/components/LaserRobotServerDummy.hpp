//
// = FILENAME
//    LaserRobotServerDummy.hpp
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
#include <Laser.hpp>

#include <cast/core/CASTComponent.hpp>

#include <libplayerc++/playerc++.h>

class LaserRobotServerDummy : virtual public Laser::LaserServer,
                              virtual public cast::CASTComponent
{
public:
  /**
   * Constructor
   */
  LaserRobotServerDummy();

  /**
   * Destructor
   */
  ~LaserRobotServerDummy();

  Robotbase::Odometry pullOdometry();

  void execMotionCommand(const ::Robotbase::MotionCommand& cmd);
  
  void registerOdometryPushClient(const Robotbase::OdometryPushClientPrx&, 
                                  Ice::Double freq);

  virtual Laser::Scan2d pullScan2d(const Ice::Current&);

  virtual void registerScan2dPushClient(const Laser::Scan2dPushClientPrx& c, 
                                        Ice::Double freq,  
                                          const Ice::Current&);
   
protected:
  virtual void configure(const std::map<std::string,std::string> & config);

  virtual void start();

  virtual void runComponent();

  virtual void stop();

protected:

  class RobotServerHelper : virtual public Robotbase::RobotbaseServer,
                            virtual public cast::CASTComponent
  {
    LaserRobotServerDummy* m_Owner;
  public:
    RobotServerHelper(LaserRobotServerDummy* owner)
      :m_Owner(owner)
    {}
    
    virtual ~RobotServerHelper() {}
    
    void runComponent() {}
    
    Robotbase::Odometry pullOdometry(const Ice::Current&)
    {
      return m_Owner->pullOdometry();
    }
    
    void execMotionCommand(const ::Robotbase::MotionCommand& cmd,
                           const Ice::Current&)
    {
      m_Owner->execMotionCommand(cmd);
    }
    
    void registerOdometryPushClient(const Robotbase::OdometryPushClientPrx& c, 
                                    Ice::Double freq,  
                                    const Ice::Current&)
    {
      m_Owner->registerOdometryPushClient(c, freq);
    }
  };

  class OdometryClient {
  public:
    Robotbase::OdometryPushClientPrx prx;
    double freq;
    cast::cdl::CASTTime lastTimePushed;
  };
  std::vector<OdometryClient> m_PushOdomClients;

  class Scan2dClient {
  public:
    Laser::Scan2dPushClientPrx prx;
    double freq;
    cast::cdl::CASTTime lastTimePushed;
  };
  std::vector<Scan2dClient> m_PushScanClients;

  std::string m_LaserServerName;
  std::string m_RobotServerName;

  /// The minimum time between two pushes [s], can be used to limit
  /// use of bandwidth. If <0 no limit is used
  double m_MinPushInterval;  

  Robotbase::Odometry m_Odom;
  Laser::Scan2d m_Scan;
};

