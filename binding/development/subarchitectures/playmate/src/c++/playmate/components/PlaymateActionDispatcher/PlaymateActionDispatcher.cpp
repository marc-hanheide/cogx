#include "PlaymateActionDispatcher.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/core/CASTUtils.hpp>
#include <manipulation/idl/Manipulation.hh>
#include <vision/idl/Vision.hh>

using namespace std;
using namespace cast;
using namespace cast::cdl;
using namespace planning::autogen;
using namespace Planner;
using namespace motivation::idl;
using namespace boost;
using namespace spatial::autogen;
using namespace Manipulation;
using namespace Binding;
using namespace BindingFeatures;
using namespace Vision;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new PlaymateActionDispatcher(_id);
  }
}


PlaymateActionDispatcher::PlaymateActionDispatcher(const string & _id) :
  WorkingMemoryAttachedComponent(_id),
  ActionDispatcher(_id)
{}



void PlaymateActionDispatcher::runComponent() {

  //HACk for the time being. need a more general solution in the long
  //run
  registerAction<ManipulationHandler>("move");


  registerAction<RepositionHandler>("reposition_left",
				    new RepositionHandler(*this, spatial::autogen::SPATIAL_LEFT));
  registerAction<RepositionHandler>("reposition_right", 
				    new RepositionHandler(*this, spatial::autogen::SPATIAL_RIGHT));
  registerAction<RepositionHandler>("reposition_near",
				    new RepositionHandler(*this, spatial::autogen::SPATIAL_PROXIMAL));


  registerAction<GoalAckHandler>("AckGoalAccepted", 
				 new GoalAckHandler(*this, motivation::idl::MOTIVE_ACK_ACCEPTED));
  registerAction<GoalAckHandler>("AckGoalRejected", 
				 new GoalAckHandler(*this, motivation::idl::MOTIVE_ACK_REJECTED));
  registerAction<GoalAckHandler>("AckGoalHoldsAlready", 
				 new GoalAckHandler(*this, motivation::idl::MOTIVE_ACK_ALREADY_HOLDS));



  registerAction<AnswerSetHandler>("tell_val_in-answer-set");

  registerAction< TellFeatureTypeHandler<BindingFeatures::Colour> >("tell_val_colour");
  registerAction< TellFeatureTypeHandler<BindingFeatures::Shape> >("tell_val_shape");
  registerAction< TellFeatureTypeHandler<BindingFeatures::Concept> >("tell_val_game");

}


bool RepositionHandler::handleAction(const std::string & _originalActionID,
				     const planning::autogen::Action & _action, 
				     const Planner::Command & _command) {

  component().log("RepositionHandler::handleAction");

  //reposition must have 2 args
  assert(_command.mapl_action.args.length() == 2);

  string targetVar(_command.mapl_action.args[0]);
  string landmarkVar(_command.mapl_action.args[1]);

  //sanity
  assert(landmarkVar != targetVar);

   //HACK assuming we know the spatial subarch at this point
  string actionSA("spatial.sa");

  const string & landmarkID(component().planningVar2ProxyID(landmarkVar, actionSA));
  if(landmarkID == landmarkVar) {
    component().println("no reverse mapping for: " + landmarkVar);
    return false;
  }

  const string & targetID(component().planningVar2ProxyID(targetVar, actionSA));
  if(targetID == targetVar) {
    component().println("no reverse mapping for: " + targetVar);
    return false;
  }


  RepositionLocation * rl = new RepositionLocation();
  rl->m_landmarkID = CORBA::string_dup(landmarkID.c_str());
  rl->m_targetID = CORBA::string_dup(targetID.c_str());
  rl->m_rel = m_rel;

  //the id for the action
  string actionID(component().newDataID());

  

  component().addToWorkingMemory(actionID, actionSA, rl, cdl::BLOCKING);
  rl = NULL;

 

  //and the type of the action
  string actionType(typeName<spatial::autogen::RepositionLocation>());

  //we need a new action to wrap the command. this just makes it
  //compatible with planning in general
  Action * action = new Action(_action);
  action->m_action.m_type = CORBA::string_dup(actionType.c_str());
  action->m_action.m_address.m_id = CORBA::string_dup(actionID.c_str());
  action->m_action.m_address.m_subarchitecture = CORBA::string_dup(actionSA.c_str());


  string newActionID(component().newDataID());

  //make sure we're listening for the response
  component().waitForAction(_originalActionID, newActionID, actionSA);

  component().addToWorkingMemory(newActionID, actionSA, action); 


  return true;
}



bool ManipulationHandler::handleAction(const std::string & _originalActionID,
				       const planning::autogen::Action & _action, 
				       const Planner::Command & _command) {

  component().log("ManipulationHandler::handleAction");


   //HACK assuming we know the manipulation subarch at this point
  string actionSA("manipulation.sa");
  //the id for the action
  string actionID(component().newDataID());

  //move must have 3 args
  assert(_command.mapl_action.args.length() == 3);


  //0 object
  //2 object
  string objectVar(_command.mapl_action.args[0]);
  string locationVar(_command.mapl_action.args[2]);

  string visionSA("vision.sa");
  string spatialSA("spatial.sa");

  //sanity
  assert(locationVar != objectVar);

  const string & locationID(component().planningVar2ProxyID(locationVar, spatialSA));
  if(locationID == locationVar) {
    component().println("no reverse mapping for: " + locationVar);
    return false;
  }

  const string & objectID(component().planningVar2ProxyID(objectVar, visionSA));
  if(objectID == objectVar) {
    component().println("no reverse mapping for: " + objectVar);
    return false;
  }

  //now get the SceneObject id of the object to be moved
  ProxyPtr objProxy(bindingHandler().loadProxy(objectID));

  if(!objProxy->hasFeature<SourceData>()) {
    component().println("No SourceData for object proxy");
    return false;
  }
  
  const SourceData & sd(objProxy->getFeature<SourceData>());
  if(strcmp(sd.m_type,typeName<SceneObject>().c_str()) != 0) {
    component().println("Incorrect SourceData for object proxy");
    return false;    
  }

  
  //now get the Location id of the target position
  ProxyPtr locationProxy(bindingHandler().loadProxy(locationID));
  if(!locationProxy->hasFeature<Location>()) {
    component().println("No Location for location proxy");
    return false;
  }
  
  const Location & loc(locationProxy->getFeature<Location>());

  component().log("ManipulationHandler::handleAction target pose: %f %f %f", loc.m_location.m_x, loc.m_location.m_y, loc.m_location.m_z);

  PickAndPlaceCmd * pnp = new PickAndPlaceCmd();
  pnp->m_objectPointer.m_address.m_id = CORBA::string_dup(sd.m_address.m_id);
  pnp->m_objectPointer.m_address.m_subarchitecture = CORBA::string_dup(visionSA.c_str());
  pnp->m_objectPointer.m_type = CORBA::string_dup(typeName<SceneObject>().c_str());
  pnp->m_targetPose.m_position = loc.m_location;
  //TODO: ask Mohan!
  //pnp->m_targetPose.m_orientation
 
  component().addToWorkingMemory(actionID, actionSA, pnp, cdl::BLOCKING);
 

  //and the type of the action
  string actionType(typeName<PickAndPlaceCmd>());

  //we need a new action to wrap the command. this just makes it
  //compatible with planning in general
  Action * action = new Action(_action);
  action->m_action.m_type = CORBA::string_dup(actionType.c_str());
  action->m_action.m_address.m_id = CORBA::string_dup(actionID.c_str());
  action->m_action.m_address.m_subarchitecture = CORBA::string_dup(actionSA.c_str());

  string newActionID(component().newDataID());

  //make sure we're listening for the response
  component().waitForAction(_originalActionID, newActionID, actionSA);

  component().addToWorkingMemory(newActionID, actionSA, action); 

  return true;
}
 
