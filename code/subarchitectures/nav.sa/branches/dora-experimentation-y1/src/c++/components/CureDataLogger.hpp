//
// = FILENAME
//   CureDataLogger.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef CureDataLogger_hpp
#define CureDataLogger_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <CureHWUtils.hpp>
#include <NavData.hpp>

#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>

// Cure includes
#include <AddressBank/FileAddress.hh>

namespace navsa {

/**
 * The purpose of this class is to store sensordata to file. It can
 * stores odometry, robot world pose and laser scans.
 *
 * @author Patric Jensfelt
 * @see
 */
class CureDataLogger : public cast::ManagedComponent, 
                       public Scan2dReceiver,
                       public OdometryReceiver
{
public:
  CureDataLogger();
  virtual ~CureDataLogger();
  
  virtual void configure(const std::map<std::string, std::string> &config);
  virtual void start();
  virtual void runComponent();

  void receiveScan2d(const Laser::Scan2d &scan);
  void receiveOdometry(const Robotbase::Odometry &odom);

protected:

  void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);

  std::string m_FilenameTime;
  Cure::FileAddress *m_Odom;
  Cure::FileAddress *m_Scan;
  Cure::FileAddress *m_World;
};

}; // namespace navsa

#endif // CureDataLogger_hpp
