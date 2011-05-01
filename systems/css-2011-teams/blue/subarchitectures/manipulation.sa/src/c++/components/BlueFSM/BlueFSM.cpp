/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <iostream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "BlueFSM.h"
#include "FrontierInterface.hpp"
#include "CureMapConversion.hpp"
#include <float.h>

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::BlueFSM();
  }
}

namespace cogx
{

  using namespace std;
  using namespace cast;
  
  BlueFSM::BlueFSM()
  {
  }
  
  void BlueFSM::configure(const map<string, string> &_config)
  {
    map<string,string>::const_iterator it;
    
  }
  
  void BlueFSM::start()
  {
    addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::objectPoseCallback));
    addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::ADD),
                    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::objectPoseCallback));

    if (!false) {
      m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
      m_mapInterface = getIceServer<FrontierInterface::LocalMapInterface>("map.manager");
    }
  }
  
  void BlueFSM::destroy()
  {
  }
  
  void BlueFSM::runComponent()
  {
    while (true)
    {
      std::ostringstream oss;
      oss << pose_.pos.x;
      log(oss.str());
      sleep(2);
    }
  }
  
  void BlueFSM::objectPoseCallback(const cdl::WorkingMemoryChange &_wmc)
  {
  //log("received objectPoseCallback");

  VisionData::VisualObjectPtr vo = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

  unsigned m_idx = std::distance(vo->identDistrib.begin(), std::max_element(vo->identDistrib.begin(), vo->identDistrib.end()));
  if (vo->identLabels.at(m_idx) == "cereals1_model")
  {
    //boost::unique_lock<boost::mutex> lock(mutex_, boost::try_to_lock_t());
    //boost::unique_lock<boost::mutex> lock(mutex_);
    pose_ = vo->pose;
  }


  //log("finished objectPoseCallback");
}

  void IcetoCureLGM(FrontierInterface::LocalGridMap icemap, CureObstMap* lgm  ){
    int lp = 0;
    for(int x = -icemap.size ; x <= icemap.size; x++){
      for(int y = -icemap.size ; y <= icemap.size; y++){ 
	(*lgm)(x,y) = (icemap.data[lp]);
	lp++;
      }
    }
  }

bool isCircleFree(const CureObstMap &cmap, double xW, double yW, double rad){
  int xiC,yiC;
  if (cmap.worldCoords2Index(xW,yW,xiC,yiC)!= 0)
    return false;

  double w = rad / cmap.getCellSize();
  int wi = int(w + 0.5);

  for (int x = xiC-wi; x <= xiC+wi; x++) {
    for (int y = yiC-wi; y <= yiC+wi; y++) {
      if (x >= -cmap.getSize() && x <= cmap.getSize() && y >= -cmap.getSize() && y <= cmap.getSize()) {
	if (hypot(x-xiC,y-yiC) < w) {
	  if (cmap(x,y) == '1' or cmap(x,y) == '2') return false;
	}
      }
    }
  }
  return true;
}

  bool BlueFSM::findGraspPoses(double objX, double objY, double theta, double halfLength,
      double bestX, double bestY){
    int currentPlaceID = m_placeInterface->getCurrentPlace()->id;
    SpatialData::PlaceIDSeq vec;
    vec.push_back(currentPlaceID);

    FrontierInterface::LocalGridMap combined_lgm;
    log("getting combined lgm");
    combined_lgm = m_mapInterface->getCombinedGridMap(vec);
    log("have combined lgm");


    CureObstMap clgm(combined_lgm.size, 0.05, '2', CureObstMap::MAP1, combined_lgm.xCenter, combined_lgm.yCenter);

    IcetoCureLGM(combined_lgm,&clgm);

    double bestAlignment = -FLT_MAX;
    
    double edge1X = objX + halfLength*cos(theta);
    double edge1Y = objY + halfLength*sin(theta);
    double edge2X = objX - halfLength*cos(theta);
    double edge2Y = objY - halfLength*sin(theta);

    vector<pair<double, double> > coordinates;
    for (int attempt = 0; attempt < 1000; attempt++) {
      const double r = 0.3 + 0.5*(double)(rand())/RAND_MAX;
      const double phi = 2*M_PI*(double)(rand())/RAND_MAX;
      double dx = r*cos(phi);
      double dy = r*sin(phi);
      double x = objX + dx;
      double y = objY + dy;

      if (isCircleFree(clgm, x, y, 0.4))  {
	double gripperDir1 = atan2(edge1Y-y, edge1X-x);
	double diff1 = gripperDir1 - theta - M_PI;
	if (cos(diff1) > bestAlignment) {
	  bestAlignment = cos(diff1);
	  bestX = edge1X;
	  bestY = edge1Y;
	}
	double gripperDir2 = atan2(edge2Y-y, edge2X-x);
	double diff2 = gripperDir2 - theta;
	if (cos(diff2) > bestAlignment) {
	  bestAlignment = cos(diff2);
	  bestX = edge2X;
	  bestY = edge2Y;
	}
      }
    }

    if (bestAlignment > -FLT_MAX) {
      return false;
    }
    return true;
  }
  
}
