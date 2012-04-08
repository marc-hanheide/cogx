//
// = FILENAME
//    RobotbaseServerFake.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Nikolaus Demmel (based on RobotbaseServerPlayer by Patric Jensfelt)
//
// = COPYRIGHT
//    Copyright (c) 2012 Nikolaus Demmel
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef RobotbaseServerFake_hpp
#define RobotbaseServerFake_hpp

#include <Ice/Ice.h>

#include <Robotbase.hpp>

#include <cast/core/CASTComponent.hpp>
#include <cast/core/CASTTimer.hpp>

namespace Robotbase 
{

/**
 * Component that implements the Robotbase::RobotbaseServer interface but
 * returns only a constant 0 fake odometry and implements commands as nops
 *
 * @author Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 */
class RobotbaseServerFake: public cast::CASTComponent
{
protected:
  class RobotbaseServerI: public Robotbase::RobotbaseServer,
                          public cast::CASTComponent 
  {
  private:
    RobotbaseServerFake *_svr;
  public:
    RobotbaseServerI(RobotbaseServerFake *svr) : _svr(svr) {}
    Robotbase::Odometry pullOdometry(const Ice::Current&);
    void execMotionCommand(const ::Robotbase::MotionCommand &cmd,
                           const Ice::Current&);
    void registerOdometryPushClient(const Robotbase::OdometryPushClientPrx&,
                                    Ice::Double interval,
                                    const Ice::Current&);
  };
  
public:
  RobotbaseServerFake();

  ~RobotbaseServerFake();

protected:
  virtual void configure(const std::map<std::string,std::string> &config);

  virtual void start();

  virtual void runComponent();

  virtual void stop();

protected:

  void execMotionCommand(const ::Robotbase::MotionCommand &cmd);

  class OdometryClient {
  public:
    Robotbase::OdometryPushClientPrx prx;
    double interval;
    cast::CASTTimer timer;
  };
  std::vector<OdometryClient> _pushClients;
  IceUtil::Mutex _pushClientMutex;

  Robotbase::Odometry _odom;

};

}; // namespace Robotbase

#endif
