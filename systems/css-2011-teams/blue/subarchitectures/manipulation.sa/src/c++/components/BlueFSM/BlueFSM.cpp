/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <iostream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "BlueFSM.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <cast/architecture.hpp>
#include "FrontierInterface.hpp"
#include "CureMapConversion.hpp"
#include <float.h>
#include "Vector3.h"
#include "Matrix33.h"

#include "Wm5.h"
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
  
  BlueFSM::BlueFSM() : m_state(INIT)
  {
  }
  
  void BlueFSM::configure(const map<string, string> &_config)
  {
    map<string,string>::const_iterator it;
    
    m_lookForObjects.push_back("cereals_weetabix");
    m_lookForObjects.push_back("cereals_choco");
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
  /*
  void
  BlueFSM::simpleCallback(const cdl::WorkingMemoryChange &wmc) {
    m_waiting = false;
  }

  void 
  BlueFSM::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID){
    VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
    rec_cmd->cmd = cmd;
    rec_cmd->label = label;
    rec_cmd->visualObjectID = visualObjectID;
    addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
  }
  */
  void BlueFSM::runComponent()
  {
    std::map<std::string, std::vector<m::Pose> > grasps;
    
    {
      std::vector<m::Pose> v;
      v.push_back(std::make_pair(m::Vector3(100,0,0), m::Quaternion(0,0,0,1)));
      v.push_back(std::make_pair(m::Vector3(-100,0,0), m::Quaternion(1,0,0,0)));
      v.push_back(std::make_pair(m::Vector3(0,0,140), m::Quaternion(0.707107,0,0.707107,0)));
      v.push_back(std::make_pair(m::Vector3(0,0,-140), m::Quaternion(0.707107,0,-0.707107,0)));

      grasps[std::string("cereals-weetabix")] = v;
    }

    while (true)
    {
      switch(m_state) {
	case INIT:
	  m_state = LOOK_CANONICAL;
	  break;
          /*
	case LOOK_CANONICAL:
	  lockComponent();
	  // Issue recognition commands
	  for (std::vector<string>::iterator it = m_lookForObjects.begin(); it != m_lookForObjects.end(); it++) {
	    addRecognizer3DCommand(VisionData::RECOGNIZE, it->c_str(), "");
	  }
	  unlockComponent();

	  m_state = DETECTING;
	  break;

	case DETECTING:
	  sleep(5);

	  m_state = DECIDE_GRASP;
	  break;
          */
	case DECIDE_GRASP:
          {
          // shortest distance between the robot and an object.
          double minDistance = 100000000; // very nice
          std::string minLabel = "";
          for (std::map<std::string, cogx::Math::Pose3>::iterator i = m_poses.begin();
               i != m_poses.end(); ++i)
            {
              if (Math::norm(i->second.pos) < minDistance)
                {
                  minDistance = Math::norm(i->second.pos);
                  minLabel = i->first;
                }
            }

	  m_state = GO_TO_PREGRASP;
	  break;
          }
	case GO_TO_PREGRASP:
	  {
	    bool success = movePregrasp(m_pregraspPose);
	    if (success) {
	      m_state = VERIFY_PREGRASP;
	    }
	    else {
	      log("Error! move pregrasp failed");
	      success = moveHome();
	      if (success) {
		m_state = INIT;
	      }
	      else {
		log("Error! Unable to return to home pose!");
		m_state = TERMINATED;
	      }
	    }
	  }
	  break;

	case ENVELOP:
	  {
	    bool success = envelop();
	    if (success) {
	      m_state = VERIFY_ENVELOP;
	    }
	    else {
	      log("Error! envelop failed");
	      success = retract();
	      if (success) {
		m_state = LOOK_CANONICAL;
	      }
	      else {
		log("Error! Unable to retract!");
		success = moveHome();
		if (success) {
		  m_state = INIT;
		}
		else {
		  log("Error! Unable to return to home pose!");
		  m_state = TERMINATED;
		}
	      }
	    }
	  }


	default:
	  break;
      }
    }
    
    /*
    ::manipulation::slice::MoveArmToPosePtr moveArmToPose(new ::manipulation::slice::MoveArmToPose());
    moveArmToPose->comp = ::manipulation::slice::COMPINIT;
    moveArmToPose->status = ::manipulation::slice::NEW;
    
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      //moveArmToPose->targetPose = pose_;
    }
    
    try {
      addToWorkingMemory(newDataID(), moveArmToPose);
    } catch (std::exception &e1) {
      log(e1.what());
    }
    
    sleep(10); // Wait for changes to happen and pray that it worked.
    */
    
    /*
    wmcr = new WorkingMemoryChangeReceiver() {
      public void workingMemoryChanged(WorkingMemoryChange _wmc) {
        commandChanged(_wmc, wmcr);
      }
    };
    
    ((CogXRunner) manipulator.getRunner()).addChangeFilter(
                                                           ChangeFilterFactory.createIDFilter(id), wmcr);

    */
  }
  
  void BlueFSM::objectPoseCallback(const cdl::WorkingMemoryChange &_wmc)
  {
    //warn("received objectPoseCallback");

    VisionData::VisualObjectPtr vo = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

    unsigned m_idx = std::distance(vo->identDistrib.begin(), std::max_element(vo->identDistrib.begin(), vo->identDistrib.end()));
//    if (vo->identLabels.at(m_idx) == "cereals1_model")
//    {
      boost::unique_lock<boost::mutex> lock(mutex_, boost::try_to_lock_t());
      //boost::unique_lock<boost::mutex> lock(mutex_);
      m_poses[vo->identLabels.at(m_idx)] = vo->pose;
//    }


      //warn("finished objectPoseCallback");
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

bool BlueFSM::movePregrasp(cogx::Math::Pose3 pregraspPose) 
{
  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = pregraspPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  removeChangeFilter(receiver);
  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::moveHome() 
{
  manipulation::slice::MoveArmToHomePositionCommandPtr cmd = new
    manipulation::slice::MoveArmToHomePositionCommand;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToHomePositionCommand>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  removeChangeFilter(receiver);
  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToHomePositionCommand>(id);

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::envelop() 
{
  cogx::Math::Pose3 targetPose;
  cogx::Math::Vector3 direction = getColumn(m_currentArmPose.rot, 0); //Get x axis of current pose

  targetPose.pos = m_currentArmPose.pos + direction * 0.05;

  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = targetPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  removeChangeFilter(receiver);
  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::retract() 
{
  cogx::Math::Pose3 targetPose;
  cogx::Math::Vector3 direction = -getColumn(m_currentArmPose.rot, 0); //Get x axis of current pose

  targetPose.pos = m_currentArmPose.pos + direction * 0.05;

  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = targetPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  removeChangeFilter(receiver);
  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}
  
}
