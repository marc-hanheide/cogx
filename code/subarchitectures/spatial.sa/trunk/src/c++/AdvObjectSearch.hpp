/*
 * AdvObjectSearch.hpp
 *
 *  Created on: Feb 15, 2010
 *      Author: aydemir
 */

#ifndef ADVOBJECTSEARCH_HPP_
#define ADVOBJECTSEARCH_HPP_

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include "Laser.hpp"
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <NavData.hpp>
#include <vector>
#include <Navigation/LocalMap.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <SensorData/SensorPose.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <SensorData/LaserScan2d.hh>
#include <Navigation/GridLineRayTracer.hh>
#include "ObjGridLineRayTracer.hh"
#include "XVector3D.h"
#include <PTZ.hpp>
#include "X11DispLocalGridMap.hh"
#include <peekabot.hh>
#include <highgui.h>




namespace spatial
{

  class AdvObjectSearch :public Scan2dReceiver,
                         public cast::ManagedComponent,
                         public OdometryReceiver

  {
  public:
    AdvObjectSearch();
    virtual ~AdvObjectSearch();
    virtual void runComponent();
    virtual void start();
  protected:
    virtual void configure(const std::map<std::string, std::string>& _config);
    void receiveScan2d(const Laser::Scan2d &castScan);
    void receiveOdometry(const Robotbase::Odometry &castOdom);
    void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
    void newPlanePointCloud(const cast::cdl::WorkingMemoryChange &objID);
    void newObjectDetected(const cast::cdl::WorkingMemoryChange &objID);
    void SavePlaneMap();
    void BuildPrior();
    void PostRecognitionCommand();
    void PostNavCommand(Cure::Pose3D position);

    void SampleGrid();
    int NextBestView();
    void SetPrior();
    void GoToNBV();
    std::vector<std::vector<int> > GetViewCones();
    std::vector<int> GetInsideViewCone(XVector3D &a, bool addall);
    void CalculateViewCone(XVector3D a, double direction, double range, double fov, XVector3D &b,XVector3D &c);
            std::vector<double> ScorebyCoverage(Cure::LocalGridMap<unsigned char> fcm );


    bool isPointSameSide(XVector3D p1,XVector3D p2,XVector3D a,XVector3D b);
    bool isPointInsideTriangle(XVector3D p,XVector3D a,XVector3D b,XVector3D c);
    void FindBoundingRectangle(XVector3D a,XVector3D b,XVector3D c,int* rectangle);
    void MeasurementUpdate(bool result);
    double ActionProbabilityPerCell(int x, int y, std::vector<int> ViewConePoints);
    void PlaneObservationUpdate(std::set<std::pair<int,int> >  NewPlanePoints) ;

  private:

    // 1. phase is table detection and then looking for objects.
    bool m_table_phase;
    bool m_usePTZ;
    int m_samplesize;
    int* m_samples;
    double* m_samplestheta;
    double m_CamRange;
    double m_fov;
    double m_MaxExplorationRange;
    IplImage* img;
    double pFree,pObs,pPlanar,pIn,pOut;
    double m_ProbGivenObjectIsPresent;
    std::vector<std::string> m_objectlist;


    IceUtil::Mutex m_Mutex;
    ptz::PTZInterfacePrx m_ptzInterface;
    NavData::RobotPose2dPtr lastRobotPose;

    peekabot::PeekabotClient m_PeekabotClient;
    peekabot::PointCloudProxy m_ProxyPrior,m_ProxyPosterior,m_ProxySeenMap;

    Cure::LocalGridMap<unsigned int>* m_lgm;
    Cure::LocalGridMap<double>* m_lgm_prior;
    Cure::LocalGridMap<double>* m_lgm_posterior;
    Cure::LocalGridMap<bool>* m_lgm_seen;
    Cure::X11DispLocalGridMap<unsigned int>* m_Dlgm;
    Cure::SensorPose m_LaserPoseR;
    Cure::SensorPose m_CamPoseR;
    Cure::TransformedOdomPoseProvider m_TOPP;
    Cure::ObjGridLineRayTracer<unsigned int>* m_Glrt;
    Cure::Pose3D m_SlamRobotPose;
    Cure::Pose3D m_currentViewPoint;
    std::vector<int>  m_CurrentViewPoint_Points;


    double m_pPlane;
    double m_pPlaneGivenObj, m_pFreeGivenObj,m_pObsGivenObj;
    double m_pPlaneGivenNotObj, m_pFreeGivenNotObj,m_pObsGivenNotObj ;

   static SpatialData::NavCommandPtr newNavCommand();
  };

}

#endif /* ADVOBJECTSEARCH_HPP_ */
