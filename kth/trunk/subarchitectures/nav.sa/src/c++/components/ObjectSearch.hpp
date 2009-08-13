#ifndef OBJECTSEARCH_HPP_
#define OBJECTSEARCH_HPP_

#include <vector>
#include <cast/architecture/ManagedComponent.hpp>
#include <NavData.hpp>
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
    int m_samplesize;
    int* m_samples;
    double* m_samplestheta;
    double m_awayfromobstacles; // in meters
    bool displayOn;
    int m_coveragetotal, m_covered;
    NavData::Completion cmp;
    bool firstscanreceived;
    bool runObjectSearch;
    unsigned int whereinplan;
    std::string id;
    Cure::LocalGridMap<unsigned int>* coveragemap;
    Cure::X11DispLocalGridMap<unsigned int>* m_Displaycoverage;
    double CoveragePercentage;
    double m_covthresh;
    struct Object{
    		std::string ObjID;
    		ObjPdf* pdf; 
    };
    struct SearchPlan{
    	std::vector<Cure::Pose3D> plan;
    	double totalcoverage;
    	std::vector<int> indexarray;
    };
    
   
   enum ObjSearchStatus {
	PLANNING,
	EXECUTINGPLAN,
	NAVCOMMANDINPROGRESS,
	NAVCOMMANDCOMPLETED,
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
	IDLE
};
     
    ObjSearchStatus m_status;
    ObjSearchCommand m_command;
    
    SearchPlan m_plan;
    double m_vpthreshold;
    Object* SuperObject;
    // fill in the blanks
    //SearchPlan m_searchplan;
    std::vector<Object*> m_objectlist;
    std::vector<int> tpoints;
    std::vector<std::vector<int> > ViewConePts;
    std::vector<Cure::Pose3D> candidatePoses;
    
    void Plan ();
    void ExecutePlan();
    void ExecuteNextInPlan();
    void InterpretCommand ();
    void PostNavCommand(Cure::Pose3D position);
    void owtNavCommand(const cast::cdl::WorkingMemoryChange & objID);
    
    
	bool GeneratePlan(double covpercent,std::vector<double> PDFsum);    
    void GenViewPoints();
    std::vector<double> IntegrateProb();
    void UpdateDisplays();
    void CalculateViewCone(XVector3D a, double direction, double range, double fov, XVector3D &b,XVector3D &c);
    bool isPointSameSide(XVector3D p1,XVector3D p2,XVector3D a,XVector3D b);
    bool isPointInsideTriangle(XVector3D p,XVector3D a,XVector3D b,XVector3D c);
    void FindBoundingRectangle(XVector3D a,XVector3D b,XVector3D c,int* rectangle);
    bool CalculateCoverage(std::vector<int> tpoints, int &covered,double treshold,Cure::LocalGridMap<unsigned int> &fcm);
    void UpdateCoverageMap();
    double ModifyCoverageMap(std::vector<int> tpoints);
    std::vector<int> GetInsideViewCone(XVector3D &a, bool addall);
    void LoadPriors();
    void InitializeObjPDF(Object* foo);
    void UpdateGlobalPDF(int index = -1);
    void ModifyGlobalPDF();
    IceUtil::Mutex m_Mutex;
    Cure::LocalGridMap<double>* m_lgm;
    Cure::LocalMap m_LMap;
  	Cure::ObjGridLineRayTracer<double>* m_Glrt;
    Cure::X11DispLocalGridMap<double>* m_Displaylgm;
    Cure::X11DispLocalGridMap<double>* m_Displaypdf;
     
 
  };

#endif /*OBJECTSEARCH_HPP_*/
