/*
 * AdvObjectSearch.hpp
 *
 *  Created on: Feb 15, 2010
 *      Author: aydemir
 */

#ifndef VISUALOBJECTSEARCH_HPP_
#define VISUALOBJECTSEARCH_HPP_


#include <cast/architecture/ManagedComponent.hpp>
#include <NavData.hpp>
#include <OdometryReceiver.hpp>
#include <Scan2dReceiver.hpp>
#include "LaserRayTracer.hh"
#include "SpatialGridMap.hh"
#include "BloxelFunctors.hh"
#include "PBVisualization.hh"
#include "ObjGridLineRayTracer.hh"
#include <Map/TransformedOdomPoseProvider.hh>
#include <SensorData/SensorPose.hh>
#include <bitset>

#include <Navigation/LocalGridMap.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <gtk/gtk.h>

namespace spatial
{

  class VisualObjectSearch : public cast::ManagedComponent,
  public Scan2dReceiver,
  public OdometryReceiver

  {
    public:
      VisualObjectSearch();
      virtual ~VisualObjectSearch();
      virtual void runComponent();
      virtual void start();
    protected:
      virtual void configure(const std::map<std::string, std::string>& _config);

    private:

      void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
      void receiveScan2d(const Laser::Scan2d &castScan);
      void receiveOdometry(const Robotbase::Odometry &castOdom);

      void SampleAndSelect();
      bool isCircleFree(double xW, double yW, double rad);
      int GetFreeSpace();

      void ReadCureMapFromFile();
      void SaveCureMapToFile();
      std::vector<double> GetViewConeSums(std::vector < std::vector<double> > samplepoints);

      SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* m_map;
      SpatialGridMap::LaserRayTracer<SpatialGridMap::GridMapData>* m_tracer;
      Cure::LocalGridMap<unsigned char>* m_lgm;
      Cure::ObjGridLineRayTracer<unsigned char>* m_Glrt;


      
      VisualPB_Bloxel* p;

      IceUtil::Mutex m_Mutex;
      Cure::Pose3D m_SlamRobotPose;
      Cure::SensorPose m_LaserPoseR;
      Cure::TransformedOdomPoseProvider m_TOPP;
      NavData::RobotPose2dPtr lastRobotPose;


      int m_samplesize;
      double m_gridsize;
      double m_cellsize;
      double m_horizangle;
      double m_vertangle;
      double m_conedepth;
      double m_minbloxel;
      double m_mapceiling;

      static void savemap( GtkWidget *widget, gpointer   data );
      static void readmap( GtkWidget *widget, gpointer   data );


      GtkWidget *window;
      GtkWidget *savebutton,*readbutton;
      GtkWidget *hbox;

      bool m_savemapmode;
  };
};
#endif /* VISUALOBJECTSEARCH_HPP_ */
