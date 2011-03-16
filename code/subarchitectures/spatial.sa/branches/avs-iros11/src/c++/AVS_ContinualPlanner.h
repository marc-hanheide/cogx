/*
 * AVS_ContinualPlanner.h
 *
 *  Created on: Mar 1, 2011
 *      Author: alper
 */

#ifndef AVS_CONTINUALPLANNER_H_
#define AVS_CONTINUALPLANNER_H_

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
		std::string bloxelMapId;
		SpatialData::SpatialRelation relation;
		std::string supportObjectId;
		std::string supportObjectCategory;
		std::string searchedObjectCategory;
		int roomId;
		int placeId;
		double getTotalProb(){
			double tmp = 0;
			for (unsigned int i =0; i < viewcones.size(); i++){
				tmp += viewcones[i].totalprob;
			}
			return tmp;
		}
	};




	typedef std::map <int, ConeGroup> MapConeType;
	AVS_ContinualPlanner();
	virtual ~AVS_ContinualPlanner();

	void start();
	void runComponent();
	void generateViewCones(SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand, std::string WMAddress);
	void processConeGroup(int id);
	std::string convertLocation2Id(SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand);
	void configure(const std::map<std::string, std::string>& _config);
	void newViewPointGenerationCommand(const cast::cdl::WorkingMemoryChange &objID);
	void IcetoCureLGM(FrontierInterface::LocalGridMap icemap, CureObstMap* lgm  );
	void receivePointCloud(FrontierInterface::WeightedPointCloudPtr cloud, double totalMass);
	 void owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID);
	 void newRobotPose(const cdl::WorkingMemoryChange &objID);
	 void PostNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype);
	 void MovePanTilt(double pan, double tilt, double tolerance);
	 void owtNavCommand(const cast::cdl::WorkingMemoryChange &objID);
	 void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label,
		std::string visualObjectID);

	 void Recognize();
	 void addARTagCommand();
	 void PostViewCone(const ViewPointGenerator::SensingAction &nbv);
	 void ViewConeUpdate(ViewPointGenerator::SensingAction viewcone, BloxelMap* map);
	 void newGroundedBelief(const cast::cdl::WorkingMemoryChange &objID);
	 std::string relationToString(SpatialData::SpatialRelation rel);
	 int GetPlaceIdFromNodeId(int nodeId);
	 int GetClosestNodeId(double x, double y, double a);
	 void newProcessConeCommand(const cast::cdl::WorkingMemoryChange &objID);

	 void receivePointCloud(BloxelMap *map, FrontierInterface::WeightedPointCloudPtr cloud, double totalMass);

private:

	 class AVSServer: public SpatialData::AVSInterface {
	 public:
	      virtual void simulateViewCones(const SpatialData::RelationalViewPointGenerationCommandPtr &cmd ,
	    		    const Ice::Current &);
	      AVS_ContinualPlanner *m_pOwner;
	      AVSServer(AVS_ContinualPlanner *owner) : m_pOwner(owner)
	      {}
	    };

		NavData::RobotPose2dPtr lastRobotPose;

	// this holds bloxel maps for each room and each bloxel has multiple pdf values for the occurrence of objects. i.e. "cupONtable", cupINroom1
	std::map<std::string, BloxelMap* > m_objectBloxelMaps;
	std::map<int, BloxelMap* > m_templateRoomBloxelMaps;
	std::map<int, CureObstMap*> m_templateRoomGridMaps;

	BloxelMap* m_currentBloxelMap;
	CureObstMap* m_currentCureObstMap;

	std::vector<std::string> generatedLocations;
	ptz::PTZInterfacePrx m_ptzInterface;

	bool m_usePTZ;
	bool m_ignoreTilt;
	bool m_usePeekabot;
	VariableNameGenerator m_namegenerator;
	int m_gridsize;
	double m_samplesize;
	double m_cellsize, m_sampleawayfromobs;
	double m_minbloxel;
	double m_conedepth;
	double m_horizangle;
	double m_minDistance;
	double m_vertangle;
	double m_tiltstep;
	double m_panstep;

	SpatialGridMap::GridMapData m_defaultBloxelCell;
	Cure::SensorPose m_LaserPoseR;
	std::string m_queryHandlerName;
	 double m_mapceiling;
	 bool m_gotPC;
	 bool  m_gotNewGenerateViewCone;

	 std::string m_PbHost;
	 VisualPB_Bloxel* pbVis;

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
	 ConeGroup m_currentConeGroup;
	 ViewPointGenerator::SensingAction m_currentViewCone;
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
