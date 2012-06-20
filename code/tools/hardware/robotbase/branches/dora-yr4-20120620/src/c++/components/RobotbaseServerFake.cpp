//
// = FILENAME
//    RobotbaseServerFake.cpp
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

#include "RobotbaseServerFake.hpp"

using namespace Robotbase;
using namespace cast;
using namespace cast::cdl;
using namespace std;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new RobotbaseServerFake();
  }
}

RobotbaseServerFake::RobotbaseServerFake()
{
  _odom.odompose.resize(1);
}

RobotbaseServerFake::~RobotbaseServerFake()
{}

void RobotbaseServerFake::configure(const map<string,string> &config)
{
  log("configure");

  Robotbase::RobotbaseServerPtr servant = new RobotbaseServerI(this);
  registerIceServer<RobotbaseServer, RobotbaseServer>(servant);
}

void RobotbaseServerFake::start()
{
  log("start");
}

void RobotbaseServerFake::stop()
{
  log("stop");
}

void RobotbaseServerFake::runComponent()
{
  log("runComponent started");

  while (isRunning()) {

    sleepComponent(100);

    _odom.time = getCASTTime();

    {
      IceUtil::Mutex::Lock lock(_pushClientMutex);

      for (unsigned int i = 0; i < _pushClients.size(); ++i)
      {
        if (!_pushClients[i].timer.isRunning() ||
            _pushClients[i].timer.split() >= _pushClients[i].interval)
        {
          log("pushed odom to client %d", i);
          _pushClients[i].timer.restart();
          _pushClients[i].prx->receiveOdometry(_odom);
        }
      }
    }

  }

  log("runComponent finished");
}


Odometry RobotbaseServerFake::RobotbaseServerI::pullOdometry(const Ice::Current&)
{
  log("pullOdometry");
  return _svr->_odom;
}

void RobotbaseServerFake::RobotbaseServerI::execMotionCommand(
    const ::Robotbase::MotionCommand &cmd, const Ice::Current&)
{
  _svr->execMotionCommand(cmd);
}

void RobotbaseServerFake::execMotionCommand(
    const ::Robotbase::MotionCommand &cmd)
{
  log("execMotionCommand (FAKE SERVER DOING NOTHING) v=%.2fm/s w=%.3frad/s",
      cmd.speed, cmd.rotspeed);
}


void RobotbaseServerFake::RobotbaseServerI::registerOdometryPushClient(
    const Robotbase::OdometryPushClientPrx &c,
    Ice::Double desiredInterval,
    const Ice::Current&)
{
  log("registerScan2dPushClient");

  OdometryClient client;
  // created this proxy as oneway allowing async data dispatch
  client.prx = c->ice_collocationOptimized(false)->ice_oneway();
  client.interval = desiredInterval;

  {
    IceUtil::Mutex::Lock lock(_svr->_pushClientMutex);
    _svr->_pushClients.push_back(client);
    _svr->log("Added push client %i", _svr->_pushClients.size()-1);
  }
}

