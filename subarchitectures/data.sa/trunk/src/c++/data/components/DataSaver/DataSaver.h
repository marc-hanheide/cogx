/*
 * DataSaver.h
 *
 *  Created on: Apr 5, 2011
 *      Author: alper
 */

#ifndef DATASAVER_H_
#define DATASAVER_H_

#include <cast/architecture/ManagedComponent.hpp>
#include "Laser.hpp"
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <PointCloudClient.h>
#include <PointCloud.hpp>
#include <PTZ.hpp>
#include <SensorData/SensorPose.hh>

class DataSaver : public Scan2dReceiver,
    public cast::ManagedComponent,
    public OdometryReceiver,
    public cast::PointCloudClient
{
public:
  DataSaver();

  virtual
  ~DataSaver();

  void
  start();

  void
  configure(const std::map<std::string, std::string>& _config);
protected:
  void
  receiveScan2d(const Laser::Scan2d &castScan);

  void
  receiveOdometry(const Robotbase::Odometry &castOdom);

  void
  getPoints(bool transformToGlobal, int imgWidth,
      PointCloud::SurfacePointSeq& points);

  void
  saveToFile();

  void
  saveToFile(PointCloud::SurfacePointSeq& points, Laser::Scan2d &castScan);

  void
  newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
private:
  ptz::PTZInterfacePrx m_ptzInterface;
  bool saveRobotPose;
  Cure::Pose3D m_SlamRobotPose;
  std::ofstream posesFile_;
};

#endif /* DATASAVER_H_ */
