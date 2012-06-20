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
#include <Scan2dReceiver.hpp>


class PushClientLaser : public Scan2dReceiver,
                         virtual public cast::CASTComponent {
public:
  /**
   * Constructor
   */
  PushClientLaser();

  /**
   * Destructor
   */
  ~PushClientLaser();
  
  void receiveScan2d(const Laser::Scan2d &scan);

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
