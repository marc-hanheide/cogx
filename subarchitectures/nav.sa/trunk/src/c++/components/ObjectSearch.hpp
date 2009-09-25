#ifndef OBJECTSEARCH_HPP_
#define OBJECTSEARCH_HPP_

#include <vector>
#include <cast/architecture/ManagedComponent.hpp>
#include <NavData.hpp>
#include <SpatialData.hpp>
#include <PTZ.hpp>
//#include <Navigation/LocalGridMap.hh>
#include "LocalGridMap.hh"
#include "ObjGridLineRayTracer.hh"
#include <SensorData/LaserScan2d.hh>
#include <Scan2dReceiver.hpp>
#include <Map/TransformedOdomPoseProvider.hh>
#include <OdometryReceiver.hpp>
#include "X11DispLocalGridMap.hh"
#include "ObjPdf.hpp"
#include "XVector3D.h"
#include <Navigation/LocalMap.hh>
class ObjectSearch : public cast::ManagedComponent,
      public Scan2dReceiver,
      public OdometryReceiver
  { 
  public:
    
    ObjectSearch();
    virtual ~ObjectSearch();
    
    void runComponent();
    void start();
    void receiveScan2d(const Laser::Scan2d &castScan);
    void receiveOdometry(const Robotbase::Odometry &castOdom);
    void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
    void newAVSCommand(const cast::cdl::WorkingMemoryChange &objID);
    long GetClosestFNode(double xW, double yW);
	std::vector<int> tpoints;
     struct SearchPlan{
    	std::vector<Cure::Pose3D> plan;
    	double totalcoverage;
    	std::vector<int> indexarray;
		std::vector<double> extracoverage;
    };
  protected:
    void configure(const std::map<std::string, std::string>& _config);
    Cure::TransformedOdomPoseProvider m_TOPP;
    Cure::Pose3D m_SlamRobotPose;
    double m_MaxExplorationRange;
    double m_CamRange;
    Cure::SensorPose m_LaserPoseR;
  private:
  	int m_gridsize; 
  	float m_cellsize;
	double m_fov;
	double m_ptustep;
    int m_samplesize;
    int* m_samples;
    double* m_samplestheta;
    double m_awayfromobstacles; // in meters
    bool displayOn;
    int m_coveragetotal, m_covered;
    NavData::Completion cmp;
    int whereinplan;
    NavData::FNodeSequence fnodeseq;
    SpatialData::PlaceIDSeq placestosearch;
    std::string id;
    Cure::LocalGridMap<unsigned int>* coveragemap;
	Cure::LocalGridMap<float>* pdf;
    Cure::X11DispLocalGridMap<unsigned int>* m_Displaycoverage;
    double CoveragePercentage;
    double m_covthresh;
    struct Object{
    		std::string ObjID;
    };
	
   
    
   
   enum ObjSearchStatus {
	PLANNING,
	EXECUTINGPLAN,
	NAVCOMMANDINPROGRESS,
	NAVCOMMANDCOMPLETED,
	RECOGNITIONINPROGRESS,
	RECOGNITIONINCOMPLETE,
	PAUSED,
	STOPPED
};

enum ObjSearchCommand {
	PLAN,
	EXECUTE,
	EXECUTENEXT,
	PAUSE,
	STOP,
	RESUME,
	TURN,
	RECOGNIZE,
	IDLE
};
	std::vector<std::string> detectedObjects;
    ObjSearchStatus m_status;
    ObjSearchCommand m_command;
    
    SearchPlan m_plan;
	double m_pan;
    double m_vpthreshold;
    // fill in the blanks
    //SearchPlan m_searchplan;
    std::vector<Object*> m_objectlist;
    
    std::vector<std::vector<int> > ViewConePts;
    std::vector<Cure::Pose3D> candidatePoses;
    void MovePanTilt(double pan,double tolerance = 0.08);
	void Recognize(); //30 degrees
    void Plan ();
    void ExecutePlan();
    void ExecuteNextInPlan();
    void InterpretCommand ();
    void PostNavCommand(Cure::Pose3D position);
    void owtNavCommand(const cast::cdl::WorkingMemoryChange & objID);
    void ObjectDetected(const cast::cdl::WorkingMemoryChange &objID);
    void PostRecognitionCommand();
	SearchPlan GeneratePlan(double covpercent,std::vector<double> PDFsum);    
	void newNavGraphNode(const cast::cdl::WorkingMemoryChange &objID);
    void GenViewPoints();
    std::vector<double> ScorebyPDF();
    void UpdateDisplays();
    void CalculateViewCone(XVector3D a, double direction, double range, double fov, XVector3D &b,XVector3D &c);
	std::vector<double> ScorebyCoverage(Cure::LocalGridMap<unsigned int> fcm );
    bool isPointSameSide(XVector3D p1,XVector3D p2,XVector3D a,XVector3D b);
    bool isPointInsideTriangle(XVector3D p,XVector3D a,XVector3D b,XVector3D c);
    void FindBoundingRectangle(XVector3D a,XVector3D b,XVector3D c,int* rectangle);
    double GetExtraCoverage(std::vector<int> tpoints, int &covered,Cure::LocalGridMap<unsigned int> &fcm, bool changefcm, std::vector<int> &rollback);
	double GetExtraCoverage(std::vector<int> tpoints, int &covered,Cure::LocalGridMap<unsigned int> &fcm);
    void Update_CoverageMap_with_GridMap();
    double ModifyCoverageMap(std::vector<int> tpoints);
    std::vector<int> GetInsideViewCone(XVector3D &a, bool addall);
    void Update_PDF_with_GridMap();
    void UpdateGlobalPDF(int index = -1);
    void ModifyGlobalPDF();
	NavData::ObjectSearchPlanPtr ConvertPlantoIce();
    IceUtil::Mutex m_Mutex;
    Cure::LocalGridMap<double>* m_lgm;
    Cure::LocalMap m_LMap;
  	Cure::ObjGridLineRayTracer<double>* m_Glrt;
    Cure::X11DispLocalGridMap<double>* m_Displaylgm;
    Cure::X11DispLocalGridMap<double>* m_Displaypdf;

    bool m_CtrlPTU;
    ptz::PTZInterfacePrx m_PTUServer;
 
  };

#endif /*OBJECTSEARCH_HPP_*/
