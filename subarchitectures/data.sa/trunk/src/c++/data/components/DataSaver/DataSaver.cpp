/*
 * DataSaver.cpp
 *
 *  Created on: Apr 5, 2011
 *      Author: alper
 */

#include "DataSaver.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <NavData.hpp>

using namespace cast;

extern "C"
{
  cast::CASTComponentPtr
  newComponent()
  {
    return new DataSaver();
  }
}

DataSaver::DataSaver(): saveRobotPose(false)
{
}

void
DataSaver::start()
{
  startPCCServerCommunication(*this);
  if (saveRobotPose){
    addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
          new MemberFunctionChangeReceiver<DataSaver>(this,
                      &DataSaver::newRobotPose));

      addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
          new MemberFunctionChangeReceiver<DataSaver>(this,
                      &DataSaver::newRobotPose));

  }
}

void DataSaver::newRobotPose(const cdl::WorkingMemoryChange &objID)
{
  NavData::RobotPose2dPtr oobj =
    getMemoryEntry<NavData::RobotPose2d>(objID.address);


  m_SlamRobotPose.setTime(Cure::Timestamp(oobj->time.s,
                                           oobj->time.us));
  m_SlamRobotPose.setX(oobj->x);
  m_SlamRobotPose.setY(oobj->y);
  m_SlamRobotPose.setTheta(oobj->theta);

  Cure::Pose3D cp = m_SlamRobotPose;
}

void
DataSaver::configure(const std::map<std::string, std::string>& _config)
{
  configureServerCommunication(_config);

  Ice::CommunicatorPtr ic = getCommunicator();

  Ice::Identity id;
  id.name = "PTZServer";
  id.category = "PTZServer";

  std::ostringstream str;
  str << ic->identityToString(id) << ":default" << " -h localhost" << " -p "
      << cast::cdl::CPPSERVERPORT;

  Ice::ObjectPrx base = ic->stringToProxy(str.str());
  m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);
}

DataSaver::~DataSaver()
{
}

void
DataSaver::receiveScan2d(const Laser::Scan2d &castScan)
{
  PointCloud::SurfacePointSeq points;
  getPoints(false, 640, points);
}

void
DataSaver::receiveOdometry(const Robotbase::Odometry &castOdom)
{

}

void
DataSaver::saveToFile(PointCloud::SurfacePointSeq& points,
    Laser::Scan2d &castScan)
{

}
