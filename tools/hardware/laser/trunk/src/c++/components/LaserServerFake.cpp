//
// = FILENAME
//    LaserServerFake.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Nikolaus Demmel (based on LaserServerPlayer by Patric Jensfelt)
//
// = COPYRIGHT
//    Copyright (c) 2012 Nikolaus Demmel
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "LaserServerFake.hpp"

using namespace Laser;
using namespace cast;
using namespace cast::cdl;
using namespace std;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new LaserServerFake();
  }
}

LaserServerFake::LaserServerFake()
{
  _scan.startAngle = 0;
  _scan.angleStep = 0.1;
  _scan.maxRange = 5;
  _scan.minRange = 0.01;
  _scan.rangeRes = 0.01;
  _scan.ranges.resize(1);
  _scan.ranges[0] = 5;
}

LaserServerFake::~LaserServerFake()
{}

void LaserServerFake::configure(const map<string,string> &config)
{
  log("configure");

  Laser::LaserServerPtr servant = new LaserServerI(this);
  registerIceServer<LaserServer, LaserServer>(servant);
}

void LaserServerFake::start()
{
  log("start");
}

void LaserServerFake::stop()
{
  log("stop");
}

void LaserServerFake::runComponent()
{
  log("runComponent started");

  while (isRunning()) {

    sleepComponent(100);

    _scan.time = getCASTTime();

    {
      IceUtil::Mutex::Lock lock(_pushClientMutex);

      for (unsigned int i = 0; i < _pushClients.size(); ++i)
      {
        if (!_pushClients[i].timer.isRunning() ||
            _pushClients[i].timer.split() >= _pushClients[i].interval)
        {
          log("pushed scan to client %d", i);
          _pushClients[i].timer.restart();
          _pushClients[i].prx->receiveScan2d(_scan);
        }
      }
    }

  }

  log("runComponent finished");
}

Scan2d LaserServerFake::LaserServerI::pullScan2d(const Ice::Current&)
{
  debug("pullScan2d");
  return _svr->_scan;
}

void LaserServerFake::LaserServerI::registerScan2dPushClient(
    const Laser::Scan2dPushClientPrx &c,
    Ice::Double desiredInterval,
    const Ice::Current&)
{
  println("registerScan2dPushClient");

  Scan2dClient client;
  client.prx = c;
  client.interval = desiredInterval;

  {
    IceUtil::Mutex::Lock lock(_svr->_pushClientMutex);
    _svr->_pushClients.push_back(client);
    _svr->log("Added push client %i", _svr->_pushClients.size()-1);
  }
}

