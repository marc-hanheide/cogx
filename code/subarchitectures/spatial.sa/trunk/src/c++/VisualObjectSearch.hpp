/* * AdvObjectSearch.hpp * *  Created on: Feb 15, 2010 *      Author: aydemir
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
#include <FrontierInterface.hpp>
#include <Navigation/LocalGridMap.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <gtk/gtk.h>

#include <PTZ.hpp>
#include "RelationEvaluation.hpp"
#include <NavData.hpp>
#include <VisionData.hpp>
#include <SpatialData.hpp>
#include "DensitySampling.hpp"

#include "XVector3D.h"
namespace spatial
{
  typedef Cure::LocalGridMap<double> CurePDFMap;
  typedef Cure::LocalGridMap<unsigned char> CureObstMap;

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
	double totalprob;
      };

      struct ObjectPairRelation{
	FrontierInterface::ObjectRelation relation;
	std::string primaryobject;
	std::string secobject;
	double prob;
	friend	bool operator== (ObjectPairRelation first, ObjectPairRelation second){
	  if (first.relation == second.relation && first.primaryobject == second.primaryobject
	      && first.secobject == second.secobject)
	    return true;
	  else
	    return false;
	}
      };

      double tryLoadStepCost(const std::vector<ObjectPairRelation> &step);
      void cacheStepCost(const std::vector<ObjectPairRelation> &step, double cost);
      
      void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
      void receiveScan2d(const Laser::Scan2d &castScan);
      void receiveOdometry(const Robotbase::Odometry &castOdom);

      void newSpatialObject(const cast::cdl::WorkingMemoryChange &objID);
      void putObjectInMap(SpatialGridMap::GridMap<SpatialGridMap::GridMapData>
	  &map, spatial::Object *object);

      vector<ObjectPairRelation> getStrategyStep(vector<string> &policy, int step);
      double GetCostForSingleStrategy(SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* tmpMap, std::string targetObject, double pout, double threshold);
      double GetStrategyCost(std::vector<std::string> policy);
      void FindBestPolicy(const vector<vector<string> > &policies);
      void owtRecognizer3DCommand(const cast::cdl::WorkingMemoryChange &objID);
      void owtNavCommand(const cast::cdl::WorkingMemoryChange &objID);
      void PostNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype);
      void PostViewCone(const SensingAction &nbv);
      void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID);
      void newVisualObject(const cast::cdl::WorkingMemoryChange &objID);
      void owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID);
      SensingAction SampleAndSelect(SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* tmpMap = 0);
      bool isCircleFree(double xW, double yW, double rad);

      void SaveCureMapToFile();
      void getStructuredStrategy(std::string strategy, std::vector<ObjectPairRelation> &singleStrategy);
      void SaveSearchPerformance(std::string result);
      void Recognize();
      void GoToNBV();
      void InterpretCommand();
      void AskForDistribution();
      int GetViewConeSums(std::vector <SensingAction > &samplepoints, SpatialGridMap::GridMap<SpatialGridMap::GridMapData> *map = 0);
      void LookforObjectWithStrategy();
      void UnsuccessfulDetection(SensingAction viewcone, SpatialGridMap::GridMap<SpatialGridMap::GridMapData> *map = 0);
      void SetCurrentTarget(const string &label);
      void InitializePDF();
      void InitializePDF(double initprob);
      void InitializePDFForObject(double initprob, const std::string &, SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* map = 0 );
      CurePDFMap* PopulateLGMap(const SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* map = 0);

      void DetectionComplete(bool isDetected, std::string detectedObject = "");
      void MovePanTilt(double pan, double tilt, double tolerance);

      /* Functions for 2D evaluation */
      std::vector<Cure::Pose3D> Sample2DGrid();
      std::vector<std::vector<pair<int,int> > > GetViewCones(std::vector<Cure::Pose3D> samples2D);
      std::vector<pair<int, int> > GetInsideViewCone(XVector3D &a, bool addall);
      void CalculateViewCone(XVector3D a, double direction, double range, double fov, XVector3D &b,XVector3D &c);
      std::vector<double> ScorebyCoverage(CureObstMap fcm );

      bool isPointSameSide(XVector3D p1,XVector3D p2,XVector3D a,XVector3D b);
      bool isPointInsideTriangle(XVector3D p,XVector3D a,XVector3D b,XVector3D c);
      void FindBoundingRectangle(XVector3D a,XVector3D b,XVector3D c,int* rectangle);
      void InitializeMaps();
      void IcetoCureLGM(FrontierInterface::LocalGridMap icemap, CureObstMap* lgm);
      void ChangeMaps(std::string roomid);
      // keyed with room id
      std::map<int, SpatialGridMap::GridMap<SpatialGridMap::GridMapData>*> m_maps;
      std::map<int, CureObstMap*> m_lgms;



      ObjectPairRelation GetSecondaryObject(std::string name);
      SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* m_map;
      SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* m_tempmap;
      SpatialGridMap::LaserRayTracer<SpatialGridMap::GridMapData>* m_tracer;
      CureObstMap* m_lgm;
      Cure::ObjGridLineRayTracer<unsigned char>* m_Glrt;

      struct ObjectRelations{
	std::string object;
	std::vector<ObjectPairRelation> relations;
      };

      enum AVSCommand{
	STOP,
	EVALUATE_POLICIES,
	ASK_FOR_DISTRIBUTION,
	RECOGNIZE,
	NEXT_NBV,
	WAITING,
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

      FrontierInterface::ObjectPriorRequestPtr m_priorreq;
      bool m_bSimulation;
      bool m_bEvaluation;
      bool gotPC;
      bool isRunComponent; 
      std::string currentTarget;
      std::string targetObject;
      std::vector<std::string> currentSearchPolicy;
      int currentPolicyStep;
      std::vector<ObjectPairRelation> searchChain;
      int searchChainPos;

      std::string m_PbHost;
      std::vector<std::string> m_objectlist;

      std::vector<std::string> m_recognizedobjects;
      AVSCommand m_command;
      int m_totalViewPoints;
      int m_waitingCount;
      SensingAction m_currentVP;
      std::vector<ObjectRelations> objectData;
      std::vector<SensingAction> exploredActions;
      VisualPB_Bloxel* pbVis;
      double m_turnangle;
      ptz::PTZInterfacePrx m_ptzInterface;
      IceUtil::Mutex m_Mutex;
      Cure::Pose3D m_SlamRobotPose;
      Cure::SensorPose m_LaserPoseR;
      Cure::TransformedOdomPoseProvider m_TOPP;
      NavData::RobotPose2dPtr lastRobotPose;

      SensingAction m_nbv;
      std::set<std::string> waitingForDetection;
      std::set<std::string> waitingForObjects;
      //      bool isWaitingForDetection;
      bool isSearchFinished;
      int viewCount;

      double m_sampleawayfromobs;
      int m_samplesize;
      double m_pout;
      double m_gridsize;
      double m_cellsize;
      double m_horizangle;
      double m_vertangle;
      double m_conedepth;
      double m_minDistance;
      double m_minbloxel;
      double m_mapceiling;
      double m_best3DConeRatio;
      double m_tiltinterval;
      static void savemap( GtkWidget *widget, gpointer   data );
      static void readmap( GtkWidget *widget, gpointer   data );

      static void selectdu( GtkWidget *widget, gpointer   data );
      static void selectdi( GtkWidget *widget, gpointer   data );
      static void selectind( GtkWidget *widget, gpointer   data );




      GtkWidget *window;
      GtkWidget *savebutton,*readbutton,*direct_uninformed, *direct_informed,*indirect;
      GtkWidget *hbox;

      bool m_showconemap;
      bool m_usePTZ;
      bool m_savemapmode;
      bool m_maploaded;
      std::string m_curemapfile;
      DensitySampler m_sampler;
  };
};
#endif /* VISUALOBJECTSEARCH_HPP_ */
