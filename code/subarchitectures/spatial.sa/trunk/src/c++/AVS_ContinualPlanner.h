/*
 * AVS_ContinualPlanner.h
 *
 *  Created on: Mar 1, 2011
 *      Author: alper
 */

#ifndef AVS_CONTINUALPLANNER_H_
#define AVS_CONTINUALPLANNER_H_

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include <cast/architecture/ManagedComponent.hpp>
#include <NavData.hpp>
#include "SpatialGridMap.hh"
#include "GridMapData.hh"
#include "BloxelFunctors.hh"
#include "FrontierInterface.hpp"
#include "VisionData.hpp"
#include "SpatialData.hpp"
#include "SpatialProperties.hpp"
#include <map>
#include <SensorData/SensorPose.hh>
#include "ConceptualData.hpp"
#include "SpatialProbabilities.hpp"
#include <Navigation/LocalGridMap.hh>
#include "DensitySampling.hpp"
#include "RelationEvaluation.hpp"
#include "XVector3D.h"
#include "Math/BinaryMatrix.hh"

#include "beliefs_cogx.hpp"
#include "beliefs.hpp"
#include "beliefs_cast.hpp"

#include "ViewPointGenerator.h"
#include <PTZ.hpp>
#include "VariableNameGenerator.h"

//#include "PBVisualization.hh"

class MainDialog;
class VisualPB_Bloxel;

namespace spatial {
class AVS_ContinualPlanner : public ManagedComponent {
public:
	typedef Cure::LocalGridMap<unsigned char> CureObstMap;
	typedef SpatialGridMap::GridMap<SpatialGridMap::GridMapData> BloxelMap;

	struct ConeGroup{
		std::vector<ViewPointGenerator::SensingAction> viewcones;
    double minAngle;
    double maxAngle;
    
		std::string bloxelMapId;
		SpatialData::SpatialRelation relation;
		std::string supportObjectId;
		std::string supportObjectCategory;
		std::string searchedObjectCategory;
		int roomId;
		int placeId;

		double getTotalProb(){
			double tmp = 0;
			for (size_t i =0; i < viewcones.size(); i++){
				tmp += viewcones[i].totalprob;
			}
			return tmp;
		}
		void scaleProbabilities(double factor) {
			for (size_t i =0; i < viewcones.size(); i++){
				viewcones[i].totalprob *= factor;
			}
		}

		bool isprocessed;

		ConeGroup(){ isprocessed=false; };
	};




	typedef std::map <int, ConeGroup> MapConeType;
	AVS_ContinualPlanner();
	virtual ~AVS_ContinualPlanner();

	void start();
	void runComponent();
	void generateViewCones(SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand, std::string WMAddress);
	void processConeGroup(int id, bool skipNav = false);
	std::string convertLocation2Id(SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand);
	void configure(const std::map<std::string, std::string>& _config);
	void newViewPointGenerationCommand(const cast::cdl::WorkingMemoryChange &objID);
	void IcetoCureLGM(SpatialData::LocalGridMap icemap, CureObstMap* lgm  );
	void receivePointCloud(FrontierInterface::WeightedPointCloudPtr cloud, double totalMass);
	 void owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID);
	 void newRobotPose(const cdl::WorkingMemoryChange &objID);
	 void PostNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype, double tol = 0);
	 void startMovePanTilt(double pan, double tilt, double tolerance);
	 void overwrittenPanTiltCommand(const cdl::WorkingMemoryChange &objID);
//	 void MovePanTilt(double pan, double tilt, double tolerance);
	 void owtNavCommand(const cast::cdl::WorkingMemoryChange &objID);
	 void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label,
		std::string visualObjectID);

	 void Recognize();
	 void addARTagCommand();
	 void PostViewCone(const ViewPointGenerator::SensingAction &nbv,int id);
	 void ViewConeUpdate( std::pair<int,ViewPointGenerator::SensingAction>  viewcone, BloxelMap* map);
	 void newGroundedBelief(const cast::cdl::WorkingMemoryChange &objID);
	 std::string relationToString(SpatialData::SpatialRelation rel);
	 int GetPlaceIdFromNodeId(int nodeId);
	 int GetClosestNodeId(double x, double y, double a);
	 void newProcessConeCommand(const cast::cdl::WorkingMemoryChange &objID);
	 void putObjectInMap(BloxelMap &map, spatial::Object *object);
	 void receivePointCloud(BloxelMap *map, FrontierInterface::WeightedPointCloudPtr cloud, double totalMass);
	 void newSpatialObject(const cast::cdl::WorkingMemoryChange &objID);
	 void displayPDF(BloxelMap map);
	 void owtARTagCommand(const cast::cdl::WorkingMemoryChange &objID);
   void owtRecognizer3DCommand(const cast::cdl::WorkingMemoryChange &objID);
   void setConeDepth(const string &label);
   ViewPointGenerator::SensingAction getRandomViewCone(ViewPointGenerator::SensingAction s);


public:
	 bool m_usePeekabot;
	 VisualPB_Bloxel* pbVis;
    bool m_sampleRandomPoints;
  double m_maxRange;
private:

    struct ForbiddenZone {
      double minX;
      double maxX;
      double minY;
      double maxY;
    };
    std::vector<ForbiddenZone> m_forbiddenZones;
  	map<int,vector<NavData::FNodePtr> > m_roomNodes;


    int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry
    peekabot::PeekabotClient m_PeekabotClient;
    std::map<int, peekabot::GroupProxy> m_ProxyViewPointsList;
    std::map<int, peekabot::PolygonProxy*> m_ProxyViewPointsPolygonsList;
    peekabot::GroupProxy* m_proxyCone;
    peekabot::PolygonProxy* m_proxyConePolygons;
    peekabot::GroupProxy m_ProxyViewPoints;
    peekabot::GroupProxy m_ProxyForbiddenMap;

    void connectPeekabot();
    void createFOV(peekabot::GroupProxy &proxy, peekabot::PolygonProxy* &proxyConeParts,
                               double fovHorizAngle, double fovVertiAngle,
                               double* color, double opacity,
                               NavData::ViewPoint viewpoint);

    void ChangeCurrentViewConeColor(double r,double g,double b);

   std::set<int> m_processedViewConeIDs;

	 class AVSServer: public SpatialData::AVSInterface {
	 public:
	      virtual void simulateViewCones(const SpatialData::RelationalViewPointGenerationCommandPtr &cmd ,
	    		    const Ice::Current &);
	      virtual void processViewCones(const int id ,const Ice::Current &);
	      AVS_ContinualPlanner *m_pOwner;
	      AVSServer(AVS_ContinualPlanner *owner) : m_pOwner(owner)
	      {}
	    };

		NavData::RobotPose2dPtr lastRobotPose;

	std::map<std::string, BloxelMap* > m_objectBloxelMaps; // this holds bloxel maps for each location i.e. <object,rel,(object2),room>
	std::map<int, BloxelMap* > m_templateRoomBloxelMaps; // template room bloxel maps to instantiate objectBloxelMaps from
	std::map<int, CureObstMap*> m_templateRoomGridMaps; // template room 2D grid maps

	BloxelMap* m_currentBloxelMap;
	CureObstMap* m_currentCureObstMap;

	std::vector<std::string> generatedLocations;
	//ptz::PTZInterfacePrx m_ptzInterface;

	std::string m_waitingForPTZCommandID;
	enum {NO_WAITING, WAITING_TO_RECOGNIZE, WAITING_TO_RETURN} m_ptzWaitingStatus;

	bool m_randomViewCones;
  bool m_usePTZ;
	bool m_ignoreTilt;
	bool m_runInSimulation;
	bool m_bUseWallPrior;
	VariableNameGenerator m_namegenerator;
	int m_gridsize;
	double m_sensingProb;
	double m_samplesize;
	double m_cellsize, m_sampleawayfromobs;
	double m_minbloxel;
	double m_conedepth;
	double m_horizangle;
	double m_minDistance;
	double m_vertangle;
	double m_tiltstep;
	double m_panstep;
	double m_coneGroupNormalization;
  double m_pdfthreshold;

	SpatialGridMap::GridMapData m_defaultBloxelCell;
	Cure::SensorPose m_LaserPoseR;
	std::string m_queryHandlerName;
	 double m_mapceiling;
	 bool m_gotPC;
	 bool  m_gotNewGenerateViewCone;

	 std::string m_PbHost;
     int m_PbPort;

	 // labels of tagged objects
	 std::vector<std::string> m_siftObjects;
	 std::vector<std::string> m_ARtaggedObjects;
	 std::vector<std::string> m_allObjects;
	 SpatialData::ProcessConeGroupPtr m_currentProcessConeGroup;
	 std::string m_processConeGroupCommandWMAddress;
	 std::string m_generateViewConesCommandWMAddress;
	 SpatialData::RelationalViewPointGenerationCommandPtr m_currentVPGenerationCommand;
	 /** ICE proxy to the QueryHandlerInterface. */
	ConceptualData::QueryHandlerServerInterfacePrx m_queryHandlerServerInterfacePrx;
	FrontierInterface::WeightedPointCloudPtr m_cloud;
	 DensitySampler m_sampler;
	 RelationEvaluator m_relationEvaluator;
	 std::map<int, ConeGroup> m_beliefConeGroups; // int is Id
	 std::map<std::string, std::string> m_fromBeliefIdtoVisualLabel;
	 std::map<int, std::string> m_coneGroupIdToBeliefId;
	 std::map<std::string, double> m_locationToBeta; // search location's so far explored region
	 std::map<std::string, std::string> m_locationToBetaWMAddress; //search location's ObjectSearchResult WMAddress

	 ConeGroup* m_currentConeGroup;
	 std::pair<int,ViewPointGenerator::SensingAction> m_currentViewCone; // Id of this SensingAction's ConeGroup and the SensingAction itself
   size_t m_currentViewConeNumber;
   int m_currentConeGroupNumber;
	 int m_coneGroupId; // Unique Id for each cone group
	MainDialog *_mainDialog;

     class NavCommandReceiver: public cast::WorkingMemoryChangeReceiver {
	      public:

		      NavCommandReceiver(AVS_ContinualPlanner & _component, SpatialData::NavCommandPtr _cmd);

		      void workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc);
	      private:
		      AVS_ContinualPlanner & m_component;
		      SpatialData::NavCommandPtr m_cmd;
     };

};
};

#endif /* AVS_CONTINUALPLANNER_H_ */

