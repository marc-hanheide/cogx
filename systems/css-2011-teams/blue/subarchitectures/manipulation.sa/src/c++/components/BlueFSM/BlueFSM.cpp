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
#include "Pose3.h"

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

  Math::Pose3 convertPose(const m::Pose& in)
  {
    Math::Pose3 p;
    for (int i = 0; i < 3; ++i)
    {
      Math::set(p.pos, i, in.first[i]);
      for (int j = 0; j < 3; ++j)
        Math::set(p.rot, i, j, m::matrixCopy(in.second)(i,j));
    }
    return p;
  }
  m::Pose convertPose(const Math::Pose3& in)
  {
    m::Pose p;
    p.first = m::Vector3(in.pos.x,in.pos.y,in.pos.z);
    m::Matrix3 m;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        m(i,j) = Math::get(in.rot, i, j);
    
    m.Orthonormalize();
    p.second = m::quaternionCopy(m);
    return p;
  }
  
  using namespace std;
  using namespace cast;
  
  BlueFSM::BlueFSM() : m_state(INIT)
  {
  }
  
  void BlueFSM::configure(const map<string, string> &_config)
  {
    map<string,string>::const_iterator it;

    m_handoverPose.pos = Math::vector3(0.4, 0.4, 0.6);
    fromAngleAxis(m_handoverPose.rot, M_PI/4, Math::vector3(0,0,1));
    
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
  
  void
  BlueFSM::simpleCallback(const cdl::WorkingMemoryChange &wmc) {
    manipulation::slice::ManipulationCommandPtr cmd =
      getMemoryEntry<manipulation::slice::ManipulationCommand>(wmc.address.id);

    if (cmd->comp == manipulation::slice::SUCCEEDED ||
	cmd->comp == manipulation::slice::FAILED) {
      m_waiting = false;
    }
  }

  void 
  BlueFSM::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID){
    VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
    rec_cmd->cmd = cmd;
    rec_cmd->label = label;
    rec_cmd->visualObjectID = visualObjectID;
    addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
  }

  void outputToFile(const std::string& s, const m::Pose& p)
  {
    std::ofstream ofs(s.c_str());
    ofs << p.first.X() << " " << p.first.Y() << " " << p.first.Z() << " " <<
    p.second.W() << " " << p.second.X() << " " << p.second.Y() << " " << p.second.Z() << std::endl;
  }
  
  std::pair<Math::Pose3, double> BlueFSM::nameless(const Math::Pose3& inRobotPose,
                                                   const Math::Pose3& inObjectPose,
                                                   const std::string& inObjectLabel,
                                                   Math::Pose3& outPregraspPose,
                                                   Math::Pose3& outEnvelopingPose,
                                                   double& outQuality) const
  {
    
    std::map<std::string, std::vector<m::Pose> > grasps;
    
    {
      std::vector<m::Pose> v;
//      v.push_back(std::make_pair(m::Vector3(.100,0,0), m::Quaternion(0,0,0,1)));
//      v.push_back(std::make_pair(m::Vector3(-.100,0,0), m::Quaternion(1,0,0,0)));
//      v.push_back(std::make_pair(m::Vector3(0,0,.140), m::Quaternion(0.707107,0,0.707107,0)));
//      v.push_back(std::make_pair(m::Vector3(0,0,-.140), m::Quaternion(0.707107,0,-0.707107,0)));

      v.push_back(std::make_pair(m::Vector3(.100,0,0), m::Quaternion(0.707107,0,0,0.707107)));
      v.push_back(std::make_pair(m::Vector3(-.100,0,0), m::Quaternion(-0.707107,0,0,0.707107)));
      v.push_back(std::make_pair(m::Vector3(0,0,-.130), m::Quaternion(0.5,0.5,0.5,0.5)));
      v.push_back(std::make_pair(m::Vector3(0,0,.130), m::Quaternion(0.5,-0.5,-0.5,0.5)));

      grasps[std::string("cereals-weetabix")] = v;
    }

    m::Pose objectPose = convertPose(inObjectPose);
    m::Pose robotPose = convertPose(inRobotPose);
    m::Pose robotRelObjectPose;
    m::project(robotRelObjectPose.first, robotRelObjectPose.second,
               robotPose.first, robotPose.second,
               objectPose.first, objectPose.second);
    
    
    outputToFile("/tmp/grasp-objectPose", objectPose);
    outputToFile("/tmp/grasp-robotPose", robotPose);
    outputToFile("/tmp/grasp-robotRelObjectPose", robotRelObjectPose);
    
    // Get the predefined grasps for that object
    
    std::vector<m::Pose> objectRelGrasps = grasps[inObjectLabel];
    
    // Find the easiest grasps in there
    
    double maxDotProduct = -1000;
    m::Pose bestGrasp = std::make_pair(m::Vector3::ZERO, m::Quaternion::IDENTITY);
    m::Pose bestBacktrackedGrasp = bestGrasp;
    for (std::vector<m::Pose>::iterator i = objectRelGrasps.begin(); i != objectRelGrasps.end(); ++i)
    {
      m::Pose robotGrasp;
      m::transform(robotGrasp.first, robotGrasp.second, robotRelObjectPose.first, robotRelObjectPose.second, i->first, i->second);
      m::Matrix3 robotGraspOri(m::matrixCopy(robotGrasp.second));
      m::Vector3 robotToGraspVector = robotGrasp.first;
      
      robotToGraspVector.Normalize();
      
      robotToGraspVector.Z() = 0;
      
      double dot = robotToGraspVector.Dot(m::Vector3(robotGraspOri.GetColumn(1)));
      if (dot > maxDotProduct)
      {
        maxDotProduct = dot;
        bestGrasp = robotGrasp;
        bestBacktrackedGrasp = bestGrasp;
        bestBacktrackedGrasp.first -= .015*m::Vector3(robotGraspOri.GetColumn(1));
      }
    }
    
    outputToFile("/tmp/grasp-bestGrasp", bestGrasp);
    outputToFile("/tmp/grasp-bestBacktrackedGrasp", bestBacktrackedGrasp);

    
    outPregraspPose = convertPose(bestBacktrackedGrasp);
    std::cerr << "pregrasp: " << outPregraspPose << std::endl;
    outEnvelopingPose = convertPose(bestGrasp);
    std::cerr << "enveloping grasp: " << outEnvelopingPose << std::endl;
    outQuality = maxDotProduct;
  }
  
  void BlueFSM::runComponent()
  {

    while (true)
    {
      switch(m_state) {
	case INIT:
	  {
	    // INIT state: Start by homing the arm, then begin searching for
	    // objects/tables
	    log("INIT");
	    m_state = LOOK_CANONICAL;

//	    bool success = moveHome();
//	    if (success) {
//	      m_state = SPINNING;
//	    }
//	    else {
//	      log("Error! Unable to return to home pose!");
//	      m_state = TERMINATED;
//	    }
	  }
	  break;
	
	case SPINNING:
	  {
	    log("SPINNING");

	    m_turnStep = 0;

	    while (m_turnStep < 8) {
	      lockComponent();
	      // Issue recognition commands
	      for (std::vector<string>::iterator it = m_lookForObjects.begin(); it != m_lookForObjects.end(); it++) {
		addRecognizer3DCommand(VisionData::RECOGNIZE, it->c_str(), "");
	      }
	      unlockComponent();

	      sleep(5);

	      m_state = DETECTING;
	    }
	  }
	  break;
	
	case LOOK_AROUND:
	  log("LOOK_AROUND");
	  m_state = LOOK_CANONICAL;
	  break;
          
	case LOOK_CANONICAL:
	  log("LOOK_CANONICAL");
	  lockComponent();
	  // Issue recognition commands
	  for (std::vector<string>::iterator it = m_lookForObjects.begin(); it != m_lookForObjects.end(); it++) {
	    addRecognizer3DCommand(VisionData::RECOGNIZE, it->c_str(), "");
	  }
	  unlockComponent();

	  m_state = DETECTING;
	  break;

	case DETECTING:
	  log("DETECTING");
	  sleep(5);

	  m_state = DECIDE_GRASP;
	  break;
          
	case DECIDE_GRASP:
	  log("DECIDE_GRASP");
          {
            // Find the label of the nearest object

            // shortest distance between the robot and an object.
            /*
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
            std::cout << minLabel << std::endl;
            */
            std::string minLabel = "cereals-weetabix";
            assert(minLabel == "cereals-weetabix");
            
            std::cerr << m_poses[minLabel] << std::endl;

            double trash;
                        
            nameless(convertPose(std::make_pair(m::Vector3::ZERO, m::Quaternion::IDENTITY)),
                     m_poses[minLabel],
                     minLabel,
                     m_pregraspPose,
                     m_envelopingPose,
                     trash);

            m_state = GO_TO_PREGRASP;
            break;
          }
        case GO_TO_PREGRASP:
	  log("GO_TO_PREGRASP");
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

	case VERIFY_PREGRASP:
	  log("VERIFY_PREGRASP");
	  {
	    m_state = ENVELOP;
	    break;
	  }

	case ENVELOP:
	  log("ENVELOP");
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

	case VERIFY_ENVELOP:
	  log("VERIFY_ENVELOP");
	  {
	    m_state = GRASP;
	    break;
	  }

	case GRASP:
	  log("GRASP");
	  {
	    bool success = grasp();
	    if (success) {
	      m_state = VERIFY_GRASP;
	    }
	    else {
	      log("Error! grasp failed");
	      success = release();
	      if (success) {
		m_state = RETRACT;
	      }
	      else {
		log("Error! Unable to release!");
		m_state = TERMINATED;
	      }
	    }
	  }

	case VERIFY_GRASP:
	  log("VERIFY_GRASP");
	  {
	    m_state = LIFT;
	    break;
	  }

	case RETRACT:
	  log("RETRACT");
	  {
	    bool success = retract();
	    if (success) {
	      m_state = LOOK_CANONICAL;
	    }
	    else {
	      log("Error! retract failed");
	      m_state = TERMINATED;
	    }
	  }

	case LIFT:
	  log("LIFT");
	  {
	    bool success = lift();
	    if (success) {
	      m_state = HANDOVER;
	    }
	    else {
	      log("Error! lift failed");
	      success = release();
	      if (success) {
		m_state = LOOK_CANONICAL;
	      }
	      else {
		log("Error! Unable to release!");
		  m_state = TERMINATED;
	      }
	    }
	  }

	case RELEASE:
	  log("RELEASE");
	  {
	    bool success = release();
	    if (success) {
	      m_state = GO_HOME;
	    }
	    else {
	      log("Error! release failed");
	      m_state = TERMINATED;
	    }
	  }

	case HANDOVER:
	  log("HANDOVER");
	  {
	    bool success = moveToHandover();
	    if (success) {
	      m_state = VERIFY_HANDOVER;
	    }
	    else {
	      log("Error! handover move failed");
	      m_state = RELEASE;
	    }
	  }

	case VERIFY_HANDOVER:
	  log("VERIFY_HANDOVER");
	  {
	    m_state = RELEASE;
	    break;
	  }
	  
	case GO_HOME:
	  log("GO_HOME");
	  {
	    bool success = moveHome();
	    if (success) {
	      m_state = TERMINATED;
	    }
	    else {
	      log("Error! move home failed");
	      m_state = TERMINATED;
	    }
	  }

	case TERMINATED:
	  log("TERMINATED");
	  log ("Terminated");
	  break;
      }

      if (m_state == TERMINATED) {
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
    m_idx = 0;
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

//  removeChangeFilter(receiver);
//  delete receiver;

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

bool BlueFSM::lift() 
{
  cogx::Math::Pose3 targetPose;

  targetPose.pos = m_currentArmPose.pos + Math::vector3(0,0,1) * 0.15;

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

bool BlueFSM::grasp() 
{
  manipulation::slice::CloseGripperCommandPtr cmd = new
    manipulation::slice::CloseGripperCommand;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->graspStatus = manipulation::slice::GRASPINGSTATUSINIT;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::CloseGripperCommand>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  removeChangeFilter(receiver);
  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::CloseGripperCommand>(id);

  return cmd->comp == manipulation::slice::SUCCEEDED &&
    cmd->graspStatus == manipulation::slice::GRASPING;
}

bool BlueFSM::release() 
{
  manipulation::slice::OpenGripperCommandPtr cmd = new
    manipulation::slice::OpenGripperCommand;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::OpenGripperCommand>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  removeChangeFilter(receiver);
  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::OpenGripperCommand>(id);

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::moveToHandover() 
{
  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = m_handoverPose;

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
