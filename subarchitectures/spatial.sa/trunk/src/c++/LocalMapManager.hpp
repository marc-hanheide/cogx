//
// = FILENAME
//    LocalMapManager.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef LocalMapManager_hpp
#define LocalMapManager_hpp
#include <cast/architecture/ManagedComponent.hpp>
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>

#include <SensorData/LaserScan2d.hh>
#include <Navigation/NavGraph.hh>
//#include <Navigation/FrontierExplorer.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <NavX/XDisplayLocalGridMap.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <Navigation/LocalMap.hh>
#include <SensorData/SensorPose.hh>
//#include <FrontierInterface.hpp>

namespace spatial {
class LocalMapManager : public cast::ManagedComponent,
  		  public OdometryReceiver,
		  public Scan2dReceiver
{
public:
  LocalMapManager();
  virtual ~LocalMapManager();

  virtual void runComponent();
  virtual void start();

protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void taskAdopted(const std::string &_taskID) {};
  virtual void taskRejected(const std::string &_taskID) {};

  double m_MaxLaserRange; 

  Cure::LocalMap m_LMap;

  Cure::GridLineRayTracer<unsigned char>* m_Glrt;
  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaylgm;

  IceUtil::Mutex m_Mutex;
  Cure::LocalGridMap<unsigned char>* m_lgm;

  Cure::TransformedOdomPoseProvider m_TOPP;

  Cure::Pose3D m_SlamRobotPose;
  Cure::Pose3D m_CurrPose;
  Cure::SensorPose m_LaserPoseR;

  std::string m_RobotServerHost;
  Robotbase::RobotbaseServerPrx m_RobotServer;
private:
  void receiveScan2d(const Laser::Scan2d &castScan);
  void receiveOdometry(const Robotbase::Odometry &castOdom);
  void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
};
}; // namespace spatial


#endif
