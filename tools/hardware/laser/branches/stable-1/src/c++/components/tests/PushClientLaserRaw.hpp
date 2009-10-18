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

#include <Laser.hpp>

#include <cast/core/CASTComponent.hpp>

class PushClientLaserRaw : public cast::CASTComponent {

public:
  /**
   * Constructor
   */
  PushClientLaserRaw();

  /**
   * Destructor
   */
  ~PushClientLaserRaw();

  virtual void receiveScan2d(const Laser::Scan2d &scan, const Ice::Current&);

protected:

  virtual void configure(const std::map<std::string,std::string> & config);
  virtual void start();
  virtual void runComponent();

protected:

  std::string m_IceServerName;
  std::string m_IceServerHost;
  int m_IceServerPort;

  Laser::LaserServerPrx m_Server;
};

class Scan2dReceiverHelper :virtual public Laser::Scan2dPushClient {
  PushClientLaserRaw * m_receiver;
public:
  Scan2dReceiverHelper(PushClientLaserRaw * _receiver) :
    m_receiver(_receiver)
  {}

  virtual void receiveScan2d(const Laser::Scan2d &scan, const Ice::Current& _crt) {
    m_receiver->receiveScan2d(scan, _crt);
  }

};
