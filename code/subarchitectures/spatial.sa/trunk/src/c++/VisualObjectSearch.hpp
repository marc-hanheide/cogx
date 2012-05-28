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
#include "AVS/src/AVSPolicyManager.hh"
#include "XVector3D.h"
#include "PBVisualization.hh"
namespace spatial {
typedef Cure::LocalGridMap<double> CurePDFMap;
typedef Cure::LocalGridMap<unsigned char> CureObstMap;

class VisualObjectSearch: public cast::ManagedComponent,
		public Scan2dReceiver,
		public OdometryReceiver,
		public EvaluatePolicy {
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

	void writeStringToFile(std::string str);
	// warning: this is only used at the start of policy execution
	// hence don't rely on it!
	int m_currentRoom;
	int m_policyRoom;
	bool m_ignoreTilt;
	struct ObjectPairRelation {
		FrontierInterface::ObjectRelation relation;
		std::string primaryobject;
		std::string secobject;
		double prob;
		friend bool operator==(ObjectPairRelation first, ObjectPairRelation second) {
			if (first.relation == second.relation && first.primaryobject
					== second.primaryobject && first.secobject == second.secobject)
				return true;
			else
				return false;
		}
	};
	//void owtARTagCommand(const cast::cdl::WorkingMemoryChange &objID); 
	void addProcessViewPointCommand();

	void addViewPointGenerationCommand();
	//    void addARTagCommand(std::string label);
	double GetGraphPathLength(double xS, double yS, double aS, double xG,
			double yG, double aG);
	int GetClosestNodeId(double x, double y, double a);
	int GetAreaId(double x, double y, double a);
	AVSPolicyManager m_policyManager;
	double GetStrategyCost(std::list<std::string> policy);
	double tryLoadStepCost(const std::vector<ObjectPairRelation> &step);
	void cacheStepCost(const std::vector<ObjectPairRelation> &step, double cost,
			std::string end = "");

	std::string printPolicy(std::vector<std::string> policy);
	void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
	void receiveScan2d(const Laser::Scan2d &castScan);
	void receiveOdometry(const Robotbase::Odometry &castOdom);

	void newViewPointGenerationCommand(
			const cast::cdl::WorkingMemoryChange &objID);
	void newProcessViewPointCommand(const cast::cdl::WorkingMemoryChange &objID);

	void newSpatialObject(const cast::cdl::WorkingMemoryChange &objID);
	void putObjectInMap(
			SpatialGridMap::GridMap<SpatialGridMap::GridMapData> &map,
			spatial::Object *object);

	vector<ObjectPairRelation> getStrategyStep(vector<string> &policy, int step);
	double GetCostForSingleStrategy(SpatialGridMap::GridMap<
			SpatialGridMap::GridMapData>* tmpMap, std::string targetObject,
			double threshold, bool ishypo);
	double GetStrategyCost(std::vector<std::string> policy);
	void owtRecognizer3DCommand(const cast::cdl::WorkingMemoryChange &objID);
	void owtNavCommand(const cast::cdl::WorkingMemoryChange &objID);
	void PostNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype);
	void PostViewCone(const SensingAction &nbv);
	void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd,
			std::string label, std::string visualObjectID);
	void newVisualObject(const cast::cdl::WorkingMemoryChange &objID);
	void owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID);
	void receivePointCloud(FrontierInterface::WeightedPointCloudPtr cloud,
			double totalMass);
	SensingAction SampleAndSelect(SpatialGridMap::GridMap<
			SpatialGridMap::GridMapData>* tmpMap = 0);
	bool isCircleFree(double xW, double yW, double rad);

	void GenerateViewPoints();
	void SaveCureMapToFile();
	void getStructuredStrategy(std::string strategy, std::vector<
			ObjectPairRelation> &singleStrategy);
	void SaveSearchPerformance(std::string result);
	void Recognize();
	void GoToNBV();
	void InterpretCommand();
	void AskForDistribution();
	int GetViewConeSums(std::vector<SensingAction> &samplepoints,
			SpatialGridMap::GridMap<SpatialGridMap::GridMapData> *map = 0);
	void LookforObjectWithStrategy();
	void UnsuccessfulDetection(SensingAction viewcone, SpatialGridMap::GridMap<
			SpatialGridMap::GridMapData> *map = 0);
	void SetCurrentTarget(const string &label);
	void InitializePDF();
	void InitializePDF(double initprob);
	void InitializePDFForObject(double initprob, const std::string &,
			SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* map = 0);
	CurePDFMap* PopulateLGMap(const SpatialGridMap::GridMap<
			SpatialGridMap::GridMapData>* map = 0);

	void DetectionComplete(bool isDetected, std::string detectedObject = "");
	void MovePanTilt(double pan, double tilt, double tolerance);
	int FindClosestPlaceID(double x, double y);
	/* Functions for 2D evaluation */
	std::vector<Cure::Pose3D> Sample2DGrid();
	std::vector<std::vector<pair<int, int> > > GetViewCones(std::vector<
			Cure::Pose3D> samples2D);
	std::vector<pair<int, int> > GetInsideViewCone(XVector3D &a, bool addall);
	void CalculateViewCone(XVector3D a, double direction, double range,
			double fov, XVector3D &b, XVector3D &c);
	std::vector<double> ScorebyCoverage(CureObstMap fcm);

	bool isPointSameSide(XVector3D p1, XVector3D p2, XVector3D a, XVector3D b);
	bool
	isPointInsideTriangle(XVector3D p, XVector3D a, XVector3D b, XVector3D c);
	void FindBoundingRectangle(XVector3D a, XVector3D b, XVector3D c,
			int* rectangle);
	void InitializeMaps(SpatialData::PlaceIDSeq placestosearch);
	void IcetoCureLGM(SpatialData::LocalGridMap icemap, CureObstMap* lgm);
	void ChangeMaps(std::string roomid);
	double GetPlaceIdFromNodeId(int nodeId);
	void setRot(double p11, double p12, double p13, double p21, double p22,
			double p23, double p31, double p32, double p33, Pose3 &p); // keyed with room id
	std::map<int, SpatialGridMap::GridMap<SpatialGridMap::GridMapData>*> m_maps;
	std::map<int, CureObstMap*> m_lgms;

	double GetPathLength(Cure::Pose3D start, Cure::Pose3D destination,
			CureObstMap* lgm = 0);

	std::string m_ProcessVPID;
	ObjectPairRelation GetSecondaryObject(std::string name);
	SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* m_map;
	SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* m_tempmap;
	SpatialGridMap::LaserRayTracer<SpatialGridMap::GridMapData>* m_tracer;
	CureObstMap* m_lgm;
	Cure::ObjGridLineRayTracer<unsigned char>* m_Glrt;

	struct ObjectRelations {
		std::string object;
		std::vector<ObjectPairRelation> relations;
	};

	enum AVSCommand {
		STOP,
		EVALUATE_POLICIES,
		ASK_FOR_DISTRIBUTION,
		GOTOROOM,
		RECOGNIZE,
		NEXT_NBV,
		WAITING,
		IDLE
	};

	SpatialData::PlaceIDSeq m_placestosearch;
	bool m_posttable;
	double m_threshold;
	bool m_usePeekabot;
	bool m_showgui;
	enum AVSStatus {
		PLANNING,
		EXECUTINGPLAN,
		NAVCOMMANDINPROGRESS,
		NAVCOMMANDCOMPLETED,
		RECOGNITIONINPROGRESS,
		RECOGNITIONCOMPLETE,
		PAUSED,
		STOPPED
	};

	double m_tilt;
	bool m_publishSimCones;
	double m_totalprob;
	FrontierInterface::ObjectPriorRequestPtr m_priorreq;
	bool m_bSimulation;
	bool m_bEvaluation;
	bool gotPC;
	bool isRunComponent;
	std::string currentTarget;
	std::string targetObject;
	std::vector<std::string> currentSearchPolicy;
	int currentPolicyStep;
	std::string failedPolicyStep;
	std::vector<ObjectPairRelation> searchChain;
	int searchChainPos;
	int maxnumberofcones;
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
	bool m_nobaseobject;
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
	static void savemap(GtkWidget *widget, gpointer data);
	static void readmap(GtkWidget *widget, gpointer data);

	static void selectdu(GtkWidget *widget, gpointer data);
	static void selectdi(GtkWidget *widget, gpointer data);
	static void selectind(GtkWidget *widget, gpointer data);

	GtkWidget *window;
	GtkWidget *savebutton, *readbutton, *direct_uninformed, *direct_informed,
			*indirect;
	GtkWidget *hbox;
	/**
	 * Local receiver type to manage nav command execution.
	 */

	SpatialData::NavCommandPtr newNavCommand();
	class NavCommandReceiver: public cast::WorkingMemoryChangeReceiver {
	public:

		NavCommandReceiver(VisualObjectSearch & _component,
				SpatialData::NavCommandPtr _cmd);

		void workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc);
	private:
		VisualObjectSearch & m_component;
		SpatialData::NavCommandPtr m_cmd;
	};

	Cure::Pose3D tablepose;
	std::string m_tablelabel;
	bool m_generateviewcones;
	std::string m_viewpointgen_id;
	bool m_showconemap;
	bool m_usePTZ;
	bool m_savemapmode;
	bool m_maploaded;
	std::string m_curemapfile;
	DensitySampler m_sampler;
};
}
;
#endif /* VISUALOBJECTSEARCH_HPP_ */
