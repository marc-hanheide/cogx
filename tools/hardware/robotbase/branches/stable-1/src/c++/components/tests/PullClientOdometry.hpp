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

class PullClientOdometry : public cast::CASTComponent {
public:
  /**
   * Constructor
   */
  PullClientOdometry();

  /**
   * Destructor
   */
  ~PullClientOdometry();

protected:

  virtual void configure(const std::map<std::string,std::string> & config);
  virtual void start();
  virtual void runComponent();

protected:

  std::string m_IceServerName;
  std::string m_IceServerHost;
  int m_IceServerPort;

  bool m_MoveRobot;

  Robotbase::RobotbaseServerPrx m_Server;
};
