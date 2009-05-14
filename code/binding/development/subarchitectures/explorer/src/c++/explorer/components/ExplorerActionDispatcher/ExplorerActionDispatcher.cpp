#include "ExplorerActionDispatcher.hpp"

#include <nav/idl/NavData.hh>

using namespace std;
using namespace cast;
using namespace cast::cdl;
using namespace NavData;
using namespace planning::autogen;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ExplorerActionDispatcher(_id);
  }
}


ExplorerActionDispatcher::ExplorerActionDispatcher(const string & _id) :
  WorkingMemoryAttachedComponent(_id),
  ActionDispatcher(_id)
{}



void ExplorerActionDispatcher::runComponent() {

  registerAction<GoalAckHandler>("AckGoalAccepted", 
				 new GoalAckHandler(*this, motivation::idl::MOTIVE_ACK_ACCEPTED));
  registerAction<GoalAckHandler>("AckGoalRejected", 
				 new GoalAckHandler(*this, motivation::idl::MOTIVE_ACK_REJECTED));
  registerAction<GoalAckHandler>("AckGoalHoldsAlready", 
				 new GoalAckHandler(*this, motivation::idl::MOTIVE_ACK_ALREADY_HOLDS));

  //registerAction<ActionSetHandler>("tell_val_in-answer-set");

  registerAction< TellFeatureTypeHandler<BindingFeatures::AreaID> >("tell_val_area-id");
 
  registerAction< TellRelationLabelHandler >("tell_val_asserted-position",
					     new TellRelationLabelHandler(*this, "position"));


  registerAction< NavCommandHandler >("move");
  registerAction< NavCommandHandler >("move_to_room");

}


bool NavCommandHandler::handleAction(const std::string & _originalActionID,
				     const planning::autogen::Action & _action, 
				     const Planner::Command & _command) {

  component().log("NavCommandHandler::handleAction");


   //HACK assuming we know the nv subarch at this point
  string actionSA("nav.sa");
  //the id for the action
  string actionID(component().newDataID());

  NavCommand *navcmd = new NavCommand();

  navcmd->m_command = NavData::GOTO_PLACE;
  navcmd->m_priority = NavData::NORMAL;
  navcmd->m_target = CORBA::string_dup(""); 
  navcmd->m_xpos = 0;
  navcmd->m_ypos = 0;
  navcmd->m_theta = 0;
  // for commands in polar coordinates    
  navcmd->m_angle = 0;  
  // for movement commands in polar coordinates or 
  // commands like "go forward 1m".  
  navcmd->m_distance = 0;    
  //for feedbacl
  navcmd->m_status = NavData::NONE;
  navcmd->m_completion = NavData::PENDING;    
  
  //if it's a move_to_room
  if(strcmp(_command.mapl_action.name,"move_to_room") == 0) {
    //a move must have 3 args
    assert(_command.mapl_action.args.length() == 3);
    //the second arg is the area id
    string areaID(_command.mapl_action.args[1]);
    component().log("NavCommandHandler::handleAction: head to area: %s", areaID.c_str());
    
    //yeah yeah I know
    //HACK trim area od stuff
    string prefix("area_id_");
    unsigned int pos = areaID.find(prefix);
    if(pos != string::npos) {
      //the lazy way
      areaID.replace(pos, prefix.size(), ""); 
    }

    //now set the actual area id
    navcmd->m_place_id = atoi(areaID.c_str());
    component().log("NavCommandHandler::handleAction: head to area: %i", navcmd->m_place_id);
  }
  else {

    //a move must have 3 args
    assert(_command.mapl_action.args.length() == 2);
     
    //HACK for test
    navcmd->m_place_id = 1;         
  }
 
  component().addToWorkingMemory(actionID, actionSA, navcmd, cdl::BLOCKING);
 

  //and the type of the action
  string actionType(typeName<NavCommand>());

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
