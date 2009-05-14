/**
 * DumbArse
 * Fuckedi-fuckedi-fuck-fuck-fuck.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#include <cmath>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include "manipulation/ManipulationGoals.h"
#include "manipulation/idl/Manipulation.hh"

#include "vision/idl/Vision.hh"
#include "vision/utils/VisionUtils.h"

#include <DumbArse.h>
// MHN:
#include <planning/idl/PlanningData.hh>

using namespace Math; using namespace Manipulation; using namespace Vision;
using namespace planning::autogen;

// if defined, draw/print some debugging information
// should be off for normal use
#define DUMBARSE_DEBUG

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new DumbArse(_id);
  }
}


DumbArse::DumbArse(const string &_id)
:  WorkingMemoryAttachedComponent(_id), ManagedProcess(_id)
{
  // working memory changes
  setReceiveXarchChangeNotifications(true);
}

DumbArse::~DumbArse()
{
}

void DumbArse::start()
{
  ManagedProcess::start();

  addChangeFilter(createGlobalTypeFilter<Vision::SceneObject>(cdl::ADD),
		  new MemberFunctionChangeReceiver<DumbArse>(this,
							      &DumbArse::handleAddObject));

  addChangeFilter(createGlobalTypeFilter<Vision::SceneObject>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<DumbArse>(this,
							     &DumbArse::handleUpdateObject));

  addChangeFilter(createGlobalTypeFilter<Vision::SceneObject>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<DumbArse>(this,
							     &DumbArse::handleDeleteObject));
  
  addChangeFilter(createGlobalTypeFilter<Vision::SceneChanged>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<DumbArse>(this,
							     &DumbArse::handleSceneChanged));

  addChangeFilter(createGlobalTypeFilter<Action>(cdl::OVERWRITE),
 		  new MemberFunctionChangeReceiver<DumbArse>(this,
 							     &DumbArse::handlePNPFinished));
}


void DumbArse::runComponent()
{
  // nothing to do
}

/**
 * Handle a new scene object.
 * If a new object appears then pick it up and move it to a nice putdown pose.
 * Adds object and pose to list. If currently no pick performed and scene is
 * static go and pick.
 */
void DumbArse::handleAddObject(const WorkingMemoryChange &changeId)
{
  const WorkingMemoryAddress &addr = changeId.m_address;

  shared_ptr<const CASTTypedData<SceneObject> > oobj =
    getWorkingMemoryEntry<SceneObject>(addr);

  if(oobj == 0)
    throw BALTException(__HERE__, "WM address points to NULL");

  log("handle add object");
  string mother_fuckin_corba_fuck_string(oobj->getData()->m_label.m_string);

  //store this for later
  visionSA = string(changeId.m_address.m_subarchitecture);

  if(mother_fuckin_corba_fuck_string != TYPE_HAND)
  {
    log("add scene object -- got valid (non-hand) object");
    // remember ID of the new object
    pickableObjects.push_back(string(addr.m_id));
    // MHN: remember a putdown pose for the new object. For now we
    // just put it down a specific offset away from where it was...
    Math::Pose3D targetPose = oobj->getData()->m_pose;

    // Randomly go to the left or the right...
    srand ( time(NULL) );
    double r((double)rand() / (double)RAND_MAX);
    if( r >= 0.5 ) {
      targetPose.m_position.m_x = 0.10; // 0.05;
      targetPose.m_position.m_y = 0.60; // 0.20;
    } else {
      targetPose.m_position.m_x = 0.10; // 0.05;
      targetPose.m_position.m_y = 0.0;  // 0.20;
    }
    putdownPoses.push_back( targetPose );
    // if we are idle and scene is static, perform pick
    // (note that we don't want to start a new pick-and-place while things in
    // the scene are moving)
    if(!performingPick()) {
      if(isSceneStatic()) {
	pickNextObject();
      }
    }
  }
  else
  {      
    // remember new hand
    log("add scene object -- got hand object");
    hands.push_back(string(addr.m_id));
    handEntered();
  }
  log("now have %d objects and %d hands", (int)pickableObjects.size(),
      (int)hands.size());
}

void DumbArse::handleUpdateObject(const WorkingMemoryChange &changeId)
{
  // we can safely ignore update:
  // - hand position does not matter, so ignore updates of hands
  // - for objects: if it moved (which it will once it is carried!), the
  //   putdown pose shall remain the same, so ignore updates of objects
}

void DumbArse::handleDeleteObject(const WorkingMemoryChange &changeId)
{
  list<string>::iterator obj = pickableObjects.begin();
  list<Pose3D>::iterator pose = putdownPoses.begin();

  log("handle delete object");
  // find deleted object in our list
  while(obj != pickableObjects.end())
  {
    if(*obj == string(changeId.m_address.m_id))
    {
      // delete object and associated putdown pose
      pickableObjects.erase(obj);
      putdownPoses.erase(pose);
      break;
    }
    else
    {
      obj++;
      pose++;
    }
  }

  // also look in the list of hands
  list<string>::iterator hand =
    find(hands.begin(), hands.end(), string(changeId.m_address.m_id));
  if(hand != hands.end())
  {
    // and delete
    hands.erase(hand);
    if(hands.empty())
      allHandsExited();
  }
  log("now have %d objects and %d hands", (int)pickableObjects.size(),
      (int)hands.size());
}


// MHN:
/**
 * Handle a finished (successfully or not) PickAndPlace command.
 * If a pick and place has finished, pick the next object.
 */
void DumbArse::handlePNPFinished(const WorkingMemoryChange &changeId)
{
  string mother_fuckin_corba_string(changeId.m_address.m_id);
  shared_ptr<const CASTTypedData<Action> > actionData =
    getWorkingMemoryEntry<Action>(mother_fuckin_corba_string);
  // see if the action was indeed a pick and place command
  if(strcmp(actionData->getData()->m_action.m_type, typeName<PickAndPlaceCmd>().c_str()) == 0)
  {
    if(actionData->getData()->m_succeeded == cast::cdl::triTrue)
      log("pick and place finished: success");
    else
      log("pick and place finished: failure");

    pickableObjects.pop_front();
    putdownPoses.pop_front();
    pursuedObjId = "";


    // if we have more pickable objects and the scene is static pick next
    // (note that we don't want to start a new pick-and-place while things in
    // the scene are moving)
    if(!pickableObjects.empty()) {
      if(isSceneStatic()) {
	//sleep for a while to let the arm finish!
	for(unsigned int i = 0; i < 5; ++i) {
	  log("%d...",i);
	  sleepProcess(1000);
	}
	pickNextObject();
      }
    }
  }
}


void DumbArse::handleSceneChanged(const WorkingMemoryChange &changeId)
{
  log("handle scene changed");
  shared_ptr<const CASTTypedData<SceneChanged> > cc =
    getWorkingMemoryEntry<SceneChanged>(changeId.m_address);
  if(cc == 0)
    throw BALTException(__HERE__, "WM address points to NULL");

  // if scene has changed (and is static again)
  if(cc->getData()->m_sceneChanged)
  {
    // if we have any pickable object and are idle pix next
    if(!pickableObjects.empty()) {
      log("scenechanged and pickable objects exist");
      if(!performingPick()) {
	log("scenechanged, pickable objects exist and not currently picking -- pick next object");
	pickNextObject();
      }
    }
  }
}



/**
 * Returns whether any change detector reports any ongoing change in the scene.
 * Note that if there are no change detectors active, then obviously we return
 * true (i.e. a static scene).
 */
bool DumbArse::isSceneStatic()
{
  string vis_subarch = "vision.sa";
  vector <shared_ptr<const CASTData<SceneChanged> > > blorgs;
  getWorkingMemoryEntries(vis_subarch, 0, blorgs);
  bool changing = false;
  for(unsigned i = 0; i < blorgs.size(); i++)
  {
    shared_ptr<const SceneChanged> sc = blorgs[i]->getData();
    if(sc->m_sceneChanging)
      changing = true;
  }
  return !changing;
}

void DumbArse::redrawGraphics3D()
{
#ifdef DUMBARSE_DEBUG
  // draw our goal poses in blue
  for(list<Pose3D>::iterator pose = putdownPoses.begin();
      pose != putdownPoses.end(); pose++)
  {
    int r, g, b;
    r = 255;
    g = 0;
    b = 255;
    drawFrame3D(pose->m_position.m_x, pose->m_position.m_y,
        pose->m_position.m_z, pose->m_orientation.m_x,
        pose->m_orientation.m_y, pose->m_orientation.m_z,
        r, g, b,  0);
  }
#endif
}


void DumbArse::redrawGraphicsText()
{
  printText("pick/placing '%s'\n", pursuedObjId.c_str());
  printText("%d pickable objects in queue:\n", pickableObjects.size());
  for(list<string>::iterator it = pickableObjects.begin();
      it != pickableObjects.end(); ++it)
  {
    printText("  %s\n", it->c_str());
  }
}


// MHN:
void DumbArse::pickNextObject()
{
  PickAndPlaceCmd *picknplace = new PickAndPlaceCmd();

  picknplace->m_objectPointer.m_address.m_id =
      CORBA::string_dup(pickableObjects.front().c_str());

  picknplace->m_objectPointer.m_address.m_subarchitecture =
      CORBA::string_dup(visionSA.c_str());

  picknplace->m_objectPointer.m_type =
    CORBA::string_dup(typeName<Vision::SceneObject>().c_str());

  picknplace->m_targetPose = putdownPoses.front();

  pursuedObjId = pickableObjects.front();

  string pnpID = newDataID();
  addToWorkingMemory<PickAndPlaceCmd>( pnpID, picknplace  /*, cdl::BLOCKING*/);

  log("added picknplace command");

  Action *action = new Action();
  action->m_action.m_type = CORBA::string_dup(typeName<PickAndPlaceCmd>().c_str());
  action->m_action.m_address.m_id = CORBA::string_dup(pnpID.c_str());
  action->m_action.m_address.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
  action->m_status = PROPOSED;
  action->m_succeeded = cast::cdl::triTrue;
  addToWorkingMemory<Action>( newDataID(), action  /*, cdl::BLOCKING*/);

  log("added action struct");
}


void DumbArse::handEntered()
{
  PauseCmd *pause = new PauseCmd();
  pause->dummy = 0;
  addToWorkingMemory<PauseCmd>( newDataID(), pause  /*, cdl::BLOCKING*/);
}

void DumbArse::allHandsExited()
{
  ResumeCmd *resume = new ResumeCmd();
  resume->dummy = 0;

  addToWorkingMemory<ResumeCmd>( newDataID(), resume  /*, cdl::BLOCKING*/);
}

