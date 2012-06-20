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

#ifndef LaserServerFake_hpp
#define LaserServerFake_hpp

#include <Ice/Ice.h>

#include <Laser.hpp>

#include <cast/core/CASTComponent.hpp>
#include <cast/core/CASTTimer.hpp>

namespace Laser {

/**
 * A server that implements the Laser::LaserServer interface but returns only a
 * constant fake scan.
 *
 * @author Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 */
class LaserServerFake : public cast::CASTComponent
{
protected:
	class LaserServerI : public Laser::LaserServer,
                       public cast::CASTComponent
  {
  private:
    LaserServerFake *_svr;
  public:
		LaserServerI(LaserServerFake *svr) : _svr(svr) {}
		Laser::Scan2d pullScan2d(const Ice::Current&);
		void registerScan2dPushClient(const Laser::Scan2dPushClientPrx&,
                                  Ice::Double interval,
                                  const Ice::Current&);
	};

public:
  LaserServerFake();

  ~LaserServerFake();

protected:
  virtual void configure(const std::map<std::string,std::string> &config);

  virtual void start();

  virtual void runComponent();

  virtual void stop();

protected:

  class Scan2dClient {
  public:
    Laser::Scan2dPushClientPrx prx;
    double interval;
    cast::CASTTimer timer;
  };
  std::vector<Scan2dClient> _pushClients;
  IceUtil::Mutex _pushClientMutex;

  Laser::Scan2d _scan;

};

}; // namespace Laser

#endif
