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


class PushClientOdometry : public OdometryReceiver,
                           virtual public cast::CASTComponent {
public:
  /**
   * Constructor
   */
  PushClientOdometry();

  /**
   * Destructor
   */
  ~PushClientOdometry();
  
  void receiveOdometry(const Robotbase::Odometry &odom);

protected:

  virtual void configure(const std::map<std::string,std::string> & config);
  virtual void start();
  virtual void runComponent();

protected:

  std::string m_IceServerName;
  std::string m_IceServerHost;
  int m_IceServerPort;

  Robotbase::RobotbaseServerPrx m_Server;
};
