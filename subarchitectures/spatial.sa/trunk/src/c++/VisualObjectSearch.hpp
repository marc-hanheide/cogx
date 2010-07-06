/*
 * AdvObjectSearch.hpp
 *
 *  Created on: Feb 15, 2010
 *      Author: aydemir
 */

#ifndef VISUALOBJECTSEARCH_HPP_
#define VISUALOBJECTSEARCH_HPP_


#include <cast/architecture/ManagedComponent.hpp>
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
#include <FrontierInterface.hpp>
#include <Navigation/LocalGridMap.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <gtk/gtk.h>

#include <NavData.hpp>
#include <VisionData.hpp>
#include <SpatialData.hpp>
#include "DensitySampling.hpp"

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

      struct SensingAction {
	std::vector<double> pos;
	double pan;
	double tilt;
      };
      enum SearchMode{
	DIRECT_UNINFORMED,
	DIRECT_INFORMED,
	INDIRECT
      };
struct ObjectPairRelation{
	FrontierInterface::ObjectRelation relation;
	std::string primaryobject;
	std::string secobject;
	double prob;
      };


void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
      void receiveScan2d(const Laser::Scan2d &castScan);
      void receiveOdometry(const Robotbase::Odometry &castOdom);
      void PostNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype);
      void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID);
      void newVisualObject(const cast::cdl::WorkingMemoryChange &objID);
      void owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID);
      SensingAction SampleAndSelect();
      bool isCircleFree(double xW, double yW, double rad);

      void ReadCureMapFromFile();
      void SaveCureMapToFile();
      void LoadSpatialRelations(std::string filename);

      void InterpretCommand();
      void AskForDistribution();
      int GetViewConeSums(std::vector <SensingAction > samplepoints);
      void LookforObjectWithStrategy(std::string name, SearchMode mode);
      ObjectPairRelation GetSecondaryObject(std::string name);
      SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* m_map;
      SpatialGridMap::LaserRayTracer<SpatialGridMap::GridMapData>* m_tracer;
      Cure::LocalGridMap<unsigned char>* m_lgm;
      Cure::ObjGridLineRayTracer<unsigned char>* m_Glrt;

      struct ObjectRelations{
	std::string object;
	std::vector<ObjectPairRelation> relations;
      };

      enum AVSCommand{
	STOP,
	ASK_FOR_DISTRIBUTION,
	RECOGNIZE,
	NEXT_NBV,
	IDLE
      };

      enum AVSStatus{
	PLANNING,
	EXECUTINGPLAN,
	NAVCOMMANDINPROGRESS,
	NAVCOMMANDCOMPLETED,
	RECOGNITIONINPROGRESS,
	RECOGNITIONCOMPLETE,
	PAUSED,
	STOPPED
      };
      std::string currentTarget;
      SearchMode currentSearchMode;
      std::vector<ObjectPairRelation> searchChain;
      int searchChainPos;

      AVSCommand m_command;
      std::vector<ObjectRelations> objectData;
      std::vector<SensingAction> exploredActions;
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

      static void selectdu( GtkWidget *widget, gpointer   data );
      static void selectdi( GtkWidget *widget, gpointer   data );
      static void selectind( GtkWidget *widget, gpointer   data );




      GtkWidget *window;
      GtkWidget *savebutton,*readbutton,*direct_uninformed, *direct_informed,*indirect;
      GtkWidget *hbox;

      bool m_savemapmode;
      bool m_maploaded;

      DensitySampler m_sampler;
  };
};
#endif /* VISUALOBJECTSEARCH_HPP_ */
