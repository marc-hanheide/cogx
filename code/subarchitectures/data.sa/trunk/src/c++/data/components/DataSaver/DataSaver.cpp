/*
 * DataSaver.cpp
 *
 *  Created on: Apr 5, 2011
 *      Author: alper
 */

#include "DataSaver.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <NavData.hpp>
#include <fstream>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <boost/filesystem.hpp>

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

void print_time (char* time_string)
{
  struct timeval now;
  int rc;
  rc= gettimeofday (&now, NULL);
  sprintf(time_string, "%u.%06u",
                  now.tv_sec, now.tv_usec);

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

  if(saveRobotPose){
  log("New RobotPose, saving to file");
  char* time_string= new char[40];
  print_time (time_string);
  char buf[256];
  sprintf(buf,"robotPoseSequence_%s", time_string);
  posesFile_.open (buf);
  posesFile_ << m_SlamRobotPose.getX() << m_SlamRobotPose.getY() << m_SlamRobotPose.getTheta() << m_SlamRobotPose.getTime().Seconds <<  m_SlamRobotPose.getTime().Microsec << std::endl;
  posesFile_.close();
  }
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


  map<string,string>::const_iterator it = _config.find("-c");
  it = _config.find("--save-robotpose");
    if (it != _config.end()) {
      saveRobotPose = true;
      log("Will save robotpose");
      boost::filesystem::create_directories("robotposes");
    }
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
