#include "ActionDispatcher.hpp"


#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/core/CASTUtils.hpp>
#include <motivation/components/BindingStateGenerator/BasicTranslators.hpp>

using namespace std;
using namespace cast;
using namespace cast::cdl;
using namespace planning::autogen;
using namespace Planner;
using namespace motivation::idl;
using namespace boost;
using namespace Binding;
using namespace BindingFeatures;


ActionDispatcher::ActionDispatcher(const string & _id) :
  WorkingMemoryAttachedComponent(_id),
  PrivilegedManagedProcess(_id),
  AbstractBindingWMRepresenter(dynamic_cast<cast::WorkingMemoryReaderProcess&>(*this)),
  m_bindingHandler(*this),
  m_motiveHandler(*this),
  m_avmReceiver(NULL),
  m_avmID("") {}


void ActionDispatcher::start() {

  ManagedProcess::start();

  addChangeFilter(createLocalTypeFilter<Action>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<ActionDispatcher>(this,
								     &ActionDispatcher::newAction));
  assert(m_avmReceiver == NULL);
  m_avmReceiver 
    = new MemberFunctionChangeReceiver<ActionDispatcher>(this, 
							 &ActionDispatcher::avmListAdded);
  addChangeFilter(createLocalTypeFilter<AddressVariableMappings>(cdl::ADD),
		  m_avmReceiver);

}

void 
ActionDispatcher::avmListAdded(const cast::cdl::WorkingMemoryChange& _wmc) {
  assert(m_avmID == "");
  m_avmID = string(_wmc.m_address.m_id);
  log("AddressVariableMappings list at: " + m_avmID);
  removeChangeFilter(m_avmReceiver, cdl::DELETE_RECEIVER);
  m_avmReceiver = NULL;
}
void 
ActionDispatcher::updateVariableMappings(const AddressVariableMappings & _avm) {
  m_planningVar2proxyID.clear();  
  for(unsigned int i = 0; i < _avm.m_mappings.length(); ++i) {    
    m_planningVar2proxyID[string(_avm.m_mappings[i].m_variable)][string(_avm.m_mappings[i].m_pointer.m_address.m_subarchitecture)] 
      = string(_avm.m_mappings[i].m_pointer.m_address.m_id);
    debug("mapped: " + string(_avm.m_mappings[i].m_variable) + " -> " + string(_avm.m_mappings[i].m_pointer.m_address.m_subarchitecture)  + " -> " + string(_avm.m_mappings[i].m_pointer.m_address.m_id));
  }
}

const std::string &
ActionDispatcher::planningVar2ProxyID(const std::string & _var,
				      const std::string & _sa) const {
  StringStringMapMap::const_iterator i = m_planningVar2proxyID.find(_var);
  if(i != m_planningVar2proxyID.end()) {
    StringStringMap::const_iterator j = i->second.find(_sa);
    if(j != i->second.end()) {
      return j->second;
    }
  }
  return _var;
}

const std::string &
ActionDispatcher::planningVar2ProxyID(const std::string & _var) const {
  StringStringMapMap::const_iterator i = m_planningVar2proxyID.find(_var);
  if(i != m_planningVar2proxyID.end()) {
    return i->second.begin()->second;
  }
  return _var;
}

void ActionDispatcher::newAction(const cdl::WorkingMemoryChange& _wmc) {
  log("ActionDispatcher::newAction");
  shared_ptr<const AddressVariableMappings> avm(getWorkingMemoryEntry<AddressVariableMappings>(m_avmID)->getData());
  updateVariableMappings(*avm);

  log("ActionDispatcher::read avms");
  

  string actionID(_wmc.m_address.m_id);
  shared_ptr<const Action> action(getWorkingMemoryEntry<Action>(actionID)->getData());

  string actionType(action->m_action.m_type);
  if(actionType != typeName<Command>()) {
    println("Cannot handle type: " + actionType);
    return;
  }

  
  shared_ptr<const Command> command(getWorkingMemoryEntry<Command>(action->m_action.m_address)->getData());

  //see if this works
  if(handleAction(actionID,
		  *action,
		  *command)) {    
    //ok, nothing generic
  } 
  else {
    println("dispatch failed: " + actionType);
    //copy original action
    Action * originalAction = new Action(*action);
  
    //set status of original action
    originalAction->m_status = planning::autogen::COMPLETE;
    originalAction->m_succeeded = cdl::triFalse;

    //overwrite original action
    overwriteWorkingMemory(actionID, originalAction);

  }
}


bool
ActionDispatcher::handleAction(const std::string & _originalActionID,
				    const planning::autogen::Action & _action, 
				    const Command & _cmd) {

  string actionName(_cmd.mapl_action.name);
  log("ActionDispatcher::handleAction: " + actionName);
  HandlerMap::iterator ah = m_handlers.find(actionName);

  if(ah != m_handlers.end()) {
    return ah->second->handleAction(_originalActionID,_action,_cmd);
  }
  else {
    log("no handler");
    return false;
  }
  

}

void
ActionDispatcher::waitForAction(const std::string & _originalActionID, 
				const std::string & _newActionID, 
				const std::string & _actionSA,
				ActionHandler * _handler) {
  
  addChangeFilter(createAddressFilter(_newActionID, _actionSA, cdl::OVERWRITE), 
		  new ActionResponseReceiver(*this,_originalActionID, _handler));
  
}

void 
ActionDispatcher::configure(map<string,string> & _config) {

  PrivilegedManagedProcess::configure(_config);

  if(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY] != "") {
    setBindingSubarchID(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY]);
  }
  else {
    log("binding subarch not specified, assuming it\'s local to monitor");
    setBindingSubarchID(m_subarchitectureID);
  }
}



void
ActionDispatcher::ActionResponseReceiver::workingMemoryChanged(const cast::cdl::WorkingMemoryChange& _wmc) {

  m_component.log("ActionDispatcher::ActionDispatcher::workingMemoryChanged: signalling planner");

  //read in dispatched action
  shared_ptr<const Action> dispatchedAction(m_component.getWorkingMemoryEntry<Action>(_wmc.m_address)->getData());
  //copy original action
  Action * originalAction = new Action(*(m_component.getWorkingMemoryEntry<Action>(m_actionID)->getData()));
  //read original command
  shared_ptr<const Command> originalCommand(m_component.getWorkingMemoryEntry<Command>(originalAction->m_action.m_address)->getData());

  //set status of original action
  originalAction->m_status = dispatchedAction->m_status;
  originalAction->m_succeeded = dispatchedAction->m_succeeded;

  //do whatever optional things the handler needs to do on completion
  if(m_actionHandler != NULL) {
    m_component.log("ActionDispatcher::ActionDispatcher::workingMemoryChanged: calling handler for completion");
    m_actionHandler->actionComplete(*originalAction,
				    *originalCommand,
				    *dispatchedAction,
				    originalAction->m_succeeded);
  }
  else {
    m_component.log("ActionDispatcher::ActionDispatcher::workingMemoryChanged: no handler for completion");
  }


  //delete dispatched action
  m_component.deleteFromWorkingMemory(_wmc.m_address);

  //overwrite original action
  m_component.overwriteWorkingMemory(m_actionID, originalAction);

  //and remove this receiver
  m_component.removeChangeFilter(this, cdl::DELETE_RECEIVER);


}


bool GoalAckHandler::handleAction(const std::string & _originalActionID,
				  const planning::autogen::Action & _action, 
				  const Planner::Command & _command) {

  
  component().log("GoalAckHandler::handleAction");

  //load the motive to get the source addresses
  string causeType(_action.m_cause.m_type);
  //sanity check
  assert(causeType == typeName<motivation::idl::Motive>());

  //get motive
  shared_ptr<const Motive> motive(component().getWorkingMemoryEntry<Motive>(_action.m_cause.m_address)->getData());


  //the action subarch comes from the cause pointer
  string actionSA(motive->m_creator);
  assert(actionSA != "");
  component().log("GoalAckHandler: sending acknowledgement to: %s", actionSA.c_str());

  //the id for the action
  string actionID(component().newDataID());

  AcknowledgeMotive * am = new AcknowledgeMotive();
  am->m_ack = m_ack;
  am->m_motiveType = motive->m_motiveType;
  am->m_motiveCause = motive->m_motiveCause;
  am->m_motiveAddress = _action.m_cause.m_address;

  component().addToWorkingMemory(actionID, actionSA, am, cdl::BLOCKING); 
 
  //and the type of the action
  string actionType(typeName<AcknowledgeMotive>());

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


bool AnswerSetHandler::handleAction(const std::string & _originalActionID,
				    const planning::autogen::Action & _action, 
				    const Planner::Command & _command) {

  component().log("AnswerSetHandler::handleAction");
  //tell the planner we succeeded in adding to answer set
  assert(_command.mapl_action.args.length() >= 2);  

  //the sa the will get the answer set
  std::string targetSAVar(_command.mapl_action.args[0]);
  std::string targetSA(component().planningVar2ProxyID(targetSAVar));
  assert(targetSAVar != targetSA);
  
  //now go through the list of answers to create the set
  AnswerSet * as = new AnswerSet();
  //this many answers, start from 1 (as 0 is target sa)
  as->m_answerSet.length(_command.mapl_action.args.length() - 1);
  for(unsigned int i = 1; i < _command.mapl_action.args.length(); ++i) {
    //get the proxy id for this sa
    std::string objectVar(_command.mapl_action.args[i]);
    std::string objectID(component().planningVar2ProxyID(objectVar, targetSA));
    //if it doesn't have one
    if(objectVar == objectID) {
      //pick any then
      objectID = component().planningVar2ProxyID(objectVar);	
      assert(objectID != objectVar);
    }
    //and copy into the set as pointer
    cast::workingMemoryPointer<BindingData::BindingProxy>(objectID,
							  component().bindingSubarchID(),
							  as->m_answerSet[i-1]);
  }

  //set is constructed

  //the id for the action
  string actionID(component().newDataID());
  component().addToWorkingMemory(actionID, targetSA, as, cdl::BLOCKING); 
 
  //and the type of the action
  string actionType(typeName<AnswerSet>());

  //we need a new action to wrap the command. this just makes it
  //compatible with planning in general
  Action * action = new Action(_action);
  action->m_action.m_type = CORBA::string_dup(actionType.c_str());
  action->m_action.m_address.m_id = CORBA::string_dup(actionID.c_str());
  action->m_action.m_address.m_subarchitecture = CORBA::string_dup(targetSA.c_str());

  string newActionID(component().newDataID());

  //make sure we're listening for the response
  component().waitForAction(_originalActionID, newActionID, targetSA, this);

  component().addToWorkingMemory(newActionID, targetSA, action); 

  return true;
}


//called when an action completes
void
AnswerSetHandler::actionComplete(planning::autogen::Action & _originalAction,
				 const Planner::Command & _originalCommand,
				 const planning::autogen::Action & _dispatchedAction, 
				 const cast::cdl::TriBool & _success) {


  component().log("AnswerSetHandler::actionComplete"); 
  //tell the planner that we've received all these OK
  _originalAction.m_addList.m_facts.length(_originalCommand.mapl_action.args.length() - 1);
  for(unsigned int i = 1; i < _originalCommand.mapl_action.args.length(); ++i) {

    
    component().log("adding (K s (in-answer-set s))"); 
    _originalAction.m_addList.m_facts[i-1].modality 
      = K_MODALITY;

    _originalAction.m_addList.m_facts[i-1].agent 
      = CORBA::string_dup(_originalCommand.mapl_action.args[0]);

    _originalAction.m_addList.m_facts[i-1].name 
      = CORBA::string_dup("in-answer-set");

    _originalAction.m_addList.m_facts[i-1].arguments.length(1);

    _originalAction.m_addList.m_facts[i-1].arguments[0]  
      = CORBA::string_dup(_originalCommand.mapl_action.args[i]);

    _originalAction.m_addList.m_facts[i-1].value 
      = CORBA::string_dup("true");
  }

  
}


bool
TellValHandler::handleRequestAction(const std::string & _targetSA,
				    const std::string & _requestSA,			    
				    const std::string & _originalActionID,
				    const planning::autogen::Action & _action, 
				    const Planner::Command & _command) {

  //this is the object the request is about
  std::string objectVar(_command.mapl_action.args[1]);
  std::string objectID(component().planningVar2ProxyID(objectVar, _targetSA));
  if(objectVar == objectID) {
    objectID = component().planningVar2ProxyID(objectVar, _requestSA);
    if(objectVar == objectID) {
      objectID = component().planningVar2ProxyID(objectVar);	
      assert(objectID != objectVar);
    }
    component().log("no %s proxy for %s using %s instead", _targetSA.c_str(), objectVar.c_str(), objectID.c_str());
  }


  //     component().log(objectVar);
  //     component().log(component().planningVar2ProxyID(objectVar));

    const std::string & requestType(getValType());

    component().log("sending request for %s for proxy for %s to sa %s", requestType.c_str(), objectID.c_str(), _targetSA.c_str());

    BindingQueries::FeatureRequest * req = new BindingQueries::FeatureRequest();
    req->m_fromSA = CORBA::string_dup(_requestSA.c_str());
    req->m_request.m_parameters.m_boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
    req->m_request.m_proxyID  = CORBA::string_dup(objectID.c_str());
    req->m_request.m_answer = cast::cdl::triIndeterminate;
    req->m_request.m_processed = false;
    instantiateFeaturePointer(req->m_request.m_featurePointer);
    

    std::string actionID(component().newDataID());
    component().addToWorkingMemory(actionID, _targetSA, req, cast::cdl::BLOCKING);
    req = NULL;

    //and the type of the action
    std::string actionType(cast::typeName<BindingQueries::FeatureRequest>());

    //we need a new action to wrap the command. this just makes it
    //compatible with planning in general
    planning::autogen::Action * action = new planning::autogen::Action(_action);
    action->m_action.m_type = CORBA::string_dup(actionType.c_str());
    action->m_action.m_address.m_id = CORBA::string_dup(actionID.c_str());
    action->m_action.m_address.m_subarchitecture = CORBA::string_dup(_targetSA.c_str());
    
    std::string newActionID(component().newDataID());

    //make sure we're listening for the response
    component().waitForAction(_originalActionID, newActionID, _targetSA, this);
    component().addToWorkingMemory(newActionID, _targetSA, action); 

    
    return true;

}


bool
TellValHandler::handleAssertionAction(const std::string & _targetSA,
				      const std::string & _originalActionID,
				      const planning::autogen::Action & _action, 
				      const Planner::Command & _command) {

  
  //this is the object the request is about
  std::string objectVar(_command.mapl_action.args[1]);
  std::string objectID(component().planningVar2ProxyID(objectVar, _targetSA));

  //if it asked then it should know!
  assert(objectVar != objectID);

  //load the object proxy
  const Binding::ProxyPtr & objectProxy(component().bindingHandler().loadProxy(objectID));

  const std::string & requestType(getValType());

  component().log("sending request for %s for proxy for %s to sa %s", requestType.c_str(), objectID.c_str(), _targetSA.c_str());

  BindingQueries::FeatureAssertion * req = new BindingQueries::FeatureAssertion();
  //use binding from now
  req->m_fromSA = CORBA::string_dup(component().bindingSubarchID().c_str());
  req->m_assertion.m_parameters.m_boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
  req->m_assertion.m_proxyID  = CORBA::string_dup(objectID.c_str());
  req->m_assertion.m_answer = cast::cdl::triIndeterminate;
  req->m_assertion.m_processed = false;

  //instantiate feature pointer with newly created feature
  instantiateFeaturePointer(req->m_assertion.m_featurePointer,
			    objectProxy,
			    _targetSA);
    

  std::string actionID(component().newDataID());
  component().addToWorkingMemory(actionID, _targetSA, req, cast::cdl::BLOCKING);
  req = NULL;
  
  //and the type of the action
  std::string actionType(cast::typeName<BindingQueries::FeatureAssertion>());

  //we need a new action to wrap the command. this just makes it
  //compatible with planning in general
  planning::autogen::Action * action = new planning::autogen::Action(_action);
  action->m_action.m_type = CORBA::string_dup(actionType.c_str());
  action->m_action.m_address.m_id = CORBA::string_dup(actionID.c_str());
  action->m_action.m_address.m_subarchitecture = CORBA::string_dup(_targetSA.c_str());
  
  std::string newActionID(component().newDataID());
  
  //make sure we're listening for the response
  component().waitForAction(_originalActionID, newActionID, _targetSA, this);
  component().addToWorkingMemory(newActionID, _targetSA, action); 
  
  
  return true;

}

bool
TellValHandler::handleAction(const std::string & _originalActionID,
			     const planning::autogen::Action & _action, 
			     const Planner::Command & _command) {
    component().log("TellValHandler::handleAction");
  
    ///mapl_action.na,e      args             //args 
    //((tell_val_colour vision_sa motmon0 )MrChips comsys_sa
    
    //tell val must have 2 args
    assert(_command.mapl_action.args.length() == 2);
    //and 2 cmd args
    assert(_command.cmd_args.length() == 2);

    //this is where the request needs to be send

    std::string targetSAVar(_command.cmd_args[1]);
    std::string targetSA(component().planningVar2ProxyID(targetSAVar));
    assert(targetSAVar != targetSA);

    //this is where the request originally came from
    std::string requestSAVar(_command.mapl_action.args[0]);    
    std::string requestSA(component().planningVar2ProxyID(requestSAVar));


    //if the subarch that askefd for info is being told
    if(requestSA == targetSA) {
      return handleAssertionAction(targetSA,_originalActionID, 
				   _action, _command);
    }
    //otherwise it's a question
    else {
      return handleRequestAction(targetSA,requestSA,_originalActionID, 
				 _action, _command);
    }



  }

  //called when an 
void
TellValHandler::actionComplete(planning::autogen::Action & _originalAction,
			       const Planner::Command & _originalCommand,
			       const planning::autogen::Action & _dispatchedAction, 
			       const cast::cdl::TriBool & _success) {


  string actionType(_dispatchedAction.m_action.m_type);
  if(actionType == cast::typeName<BindingQueries::FeatureAssertion>()) {

    //TODO clean up feature pointerss
    //...


    //don't do anything else on failure
    if(_success == cast::cdl::triFalse) {
      return;
    }

    //so now add the fact that targetSAVar does not know about objectVar
    std::string targetSAVar(_originalCommand.cmd_args[1]);
    std::string objectVar(_originalCommand.mapl_action.args[1]);
    std::string featureType(toPlanningFeature(getValType()));
    
    _originalAction.m_addList.m_facts.length(1);
    _originalAction.m_addList.m_facts[0].modality = K_MODALITY;
    _originalAction.m_addList.m_facts[0].agent = CORBA::string_dup(targetSAVar.c_str());
    _originalAction.m_addList.m_facts[0].name 
      = CORBA::string_dup(featureType.c_str());
    _originalAction.m_addList.m_facts[0].arguments.length(1);
    _originalAction.m_addList.m_facts[0].arguments[0]  
      = CORBA::string_dup(objectVar.c_str());
    _originalAction.m_addList.m_facts[0].value = CORBA::string_dup("true");

  }
  else {
    assert(actionType == cast::typeName<BindingQueries::FeatureRequest>());
    //don't do anything on success
    if(_success == cast::cdl::triTrue) {
      return;
    }
  


  //aslo add the knowledge state

  
  //on failure need to add that this guy doesn't know about the thing
  //it was asked about
  string maplCommand(_originalCommand.mapl_action.name);
  component().log("handling failure for %s", maplCommand.c_str());

  //replace tell_val_ with not_knows_about__ to get fact for failure
  //HACK trim destination stuff
  string prefix("tell_val_");
  unsigned int pos = maplCommand.find(prefix);
  if(pos != string::npos) {
    //the lazy way
    maplCommand.replace(pos, prefix.size(), "not_knows_about__"); 
  }

  //this was where the request was sent
  std::string targetSAVar(_originalCommand.cmd_args[1]);
  //this is the object the request was about
  std::string objectVar(_originalCommand.mapl_action.args[1]);
  
  //so now add the fact that targetSAVar does not know about objectVar
  component().log("adding failure fact: %s", maplCommand.c_str());
  _originalAction.m_addList.m_facts.length(1);
  _originalAction.m_addList.m_facts[0].modality = NO_MODALITY;
  _originalAction.m_addList.m_facts[0].agent = CORBA::string_dup("");
  _originalAction.m_addList.m_facts[0].name 
    = CORBA::string_dup(maplCommand.c_str());
  _originalAction.m_addList.m_facts[0].arguments.length(2);
  _originalAction.m_addList.m_facts[0].arguments[0]  
    = CORBA::string_dup(targetSAVar.c_str());
  _originalAction.m_addList.m_facts[0].arguments[1]  
    = CORBA::string_dup(objectVar.c_str());
  _originalAction.m_addList.m_facts[0].value = CORBA::string_dup("true");
  }

}



void TellRelationLabelHandler::instantiateFeaturePointer(BindingData::FeaturePointer & _fp) {
  //create a relation label feature 
  BindingFeatures::RelationLabel * rl = new BindingFeatures::RelationLabel();
  rl->m_label = CORBA::string_dup(m_label.c_str());
  rl->m_parent.m_truthValue = BindingFeaturesCommon::POSITIVE;
  rl->m_parent.m_immediateProxyID = CORBA::string_dup("");
  
  //and write to binding wm
  std::string featureID(component().newDataID());
  component().addToWorkingMemory(featureID,component().bindingSubarchID(), 
				 rl, cast::cdl::BLOCKING);
  rl = NULL;
  
  _fp.m_type = CORBA::string_dup(getValType().c_str());
  _fp.m_address = CORBA::string_dup(featureID.c_str());
  _fp.m_immediateProxyID = CORBA::string_dup("");
}

