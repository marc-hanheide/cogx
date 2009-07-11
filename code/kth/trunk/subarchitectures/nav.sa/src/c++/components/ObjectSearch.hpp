#ifndef OBJECTSEARCH_HPP_
#define OBJECTSEARCH_HPP_

#include <vector>
#include <cast/architecture/ManagedComponent.hpp>
#include <NavData.hpp>
//#include <Navigation/LocalGridMap.hh>
#include "XLocalGridMap.hh"
#include "ObjGridLineRayTracer.hh"
#include <SensorData/LaserScan2d.hh>
#include <Scan2dReceiver.hpp>
#include <Map/TransformedOdomPoseProvider.hh>
#include <OdometryReceiver.hpp>
#include "XDisplayLocalGridMap.hh"
#include "ObjPdf.hpp"
#include "XVector3D.h"

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
    bool displayOn;
    int highestVCindex;
    int coveragetotal, covered;
    NavData::NavCommandPtr cmd;
    NavData::Completion cmp;
    bool tasktoggle;
    bool firstscanreceived;
    bool runObjectSearch;
    std::string id;
    Cure::XLocalGridMap<unsigned int>* coveragemap;
    Cure::XDisplayLocalGridMap<unsigned int>* m_Displaycoverage;
    double CoveragePercentage;
    double coveragetreshold;
    struct Object{
    		std::string ObjID;
    		ObjPdf* pdf; 
    };
    
    Object* SuperObject;
    // fill in the blanks
    
    std::vector<Object*> m_objectlist;
    std::vector<int> tpoints;
    std::vector<std::vector<int> > ViewConePts;
    std::vector<Cure::Pose3D> candidatePoses;
    std::vector<Cure::Pose3D> SearchPlan;
    void owtNavCommand(const cast::cdl::WorkingMemoryChange & objID);
    void Commander(); 
    
    
    void GenViewPoints();
    void SelectBestView();
    void UpdateDisplays();
    void CalculateViewCone(XVector3D a, double direction, double range, double fov, XVector3D &b,XVector3D &c);
    bool isPointSameSide(XVector3D p1,XVector3D p2,XVector3D a,XVector3D b);
    bool isPointInsideTriangle(XVector3D p,XVector3D a,XVector3D b,XVector3D c);
    void FindBoundingRectangle(XVector3D a,XVector3D b,XVector3D c,int* rectangle);
    
    void UpdateCoverageMap();
    double ModifyCoverageMap(std::vector<int> tpoints,bool hypothetical = false);
    
    void LoadPriors();
    void InitializeObjPDF(Object* foo);
    void UpdateGlobalPDF(int index = -1);
    void ModifyGlobalPDF();
    
    Cure::XLocalGridMap<double>* m_lgm;
  	Cure::ObjGridLineRayTracer<double>* m_Glrt;
    Cure::XDisplayLocalGridMap<double>* m_Displaylgm;
    Cure::XDisplayLocalGridMap<double>* m_Displaypdf;
 
  };

#endif /*OBJECTSEARCH_HPP_*/
