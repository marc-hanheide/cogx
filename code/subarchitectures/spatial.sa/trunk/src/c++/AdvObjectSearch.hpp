/*
 * AdvObjectSearch.hpp
 *
 *  Created on: Feb 15, 2010
 *      Author: aydemir
 */

#ifndef ADVOBJECTSEARCH_HPP_
#define ADVOBJECTSEARCH_HPP_

#include "Laser.hpp"
#include <Scan2dReceiver.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include "AdvObjectSearch.hpp"
#include <Navigation/LocalMap.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <SensorData/SensorPose.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <SensorData/LaserScan2d.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <NavData.hpp>

namespace spatial
{

  class AdvObjectSearch :public Scan2dReceiver,
                         public cast::ManagedComponent

  {
  public:
    AdvObjectSearch();
    virtual ~AdvObjectSearch();

    virtual void runComponent();
    virtual void start();
  protected:
    virtual void configure(const std::map<std::string, std::string>& _config);
    void receiveScan2d(const Laser::Scan2d &castScan);
    void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
    void newPlanePointCloud(const cast::cdl::WorkingMemoryChange &objID);
    void SavePlaneMap();
  private:


    Cure::SensorPose m_LaserPoseR;
    Cure::SensorPose m_CamPoseR;
    Cure::TransformedOdomPoseProvider m_TOPP;
    IceUtil::Mutex m_Mutex;
    Cure::GridLineRayTracer<unsigned int>* m_Glrt;
    Cure::Pose3D m_SlamRobotPose;
    NavData::RobotPose2dPtr lastRobotPose;
    Cure::LocalGridMap<unsigned int>* m_lgm;
    // 1. phase is table detection and then looking for objects.
    bool m_table_phase;
  };

}

#endif /* ADVOBJECTSEARCH_HPP_ */
