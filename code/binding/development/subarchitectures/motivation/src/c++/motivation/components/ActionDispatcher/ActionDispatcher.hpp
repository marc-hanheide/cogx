#ifndef ACTION_DISPATCHER_H_
#define ACTION_DISPATCHER_H_

#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include <cast/core/CASTDataCache.hpp>
#include <motivation/idl/MotivationData.hh>
#include <planning/idl/PlanningData.hh>
#include <planning/idl/Planner.hh>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <binding/utils/GraphLoader.hpp>
#include <binding/idl/BindingQueries.hh>
#include <binding/idl/BindingFeatures.hh>
#include <binding/idl/BindingFeaturesCommon.hh>


class ActionHandler;



/**
 * A component that receives actions from the planner then writes them
 * to the specific places. This is a slightly dumb way of doing it,
 * but it allows us to add hacks as and when they're needed ;)
 */
class ActionDispatcher : 
  public cast::PrivilegedManagedProcess,
  public Binding::AbstractBindingWMRepresenter {
  
private:
  typedef cast::StringMap<ActionHandler *>::map HandlerMap;
  typedef cast::StringMap<std::string>::map StringStringMap;
  typedef cast::StringMap<StringStringMap>::map StringStringMapMap;
  
protected:
  class ActionResponseReceiver : public cast::WorkingMemoryChangeReceiver {
  public:
    ActionResponseReceiver(ActionDispatcher & _component,
			   const std::string & _originalActionID,
			   ActionHandler * _handler) :
      m_component(_component), 
      m_actionID(_originalActionID),
      m_actionHandler(_handler) {}
    
    void
    workingMemoryChanged(const cast::cdl::WorkingMemoryChange& _wmc);
    
    
  private:
    ActionDispatcher & m_component;
    std::string m_actionID;
    ActionHandler * m_actionHandler;
     
  };

  Binding::BindingGraphHandler m_bindingHandler;
  Binding::BindingGraphHandler m_motiveHandler;

public:
  
  ActionDispatcher(const std::string & _id);
  virtual ~ActionDispatcher() {}

  Binding::BindingGraphHandler & bindingHandler() {
    return m_bindingHandler;
  }

  Binding::BindingGraphHandler & motiveHandler() {
    return m_motiveHandler;
  }

  /*
   * Given a plannign var and the subarchitecture it's being used in,
   * returns the related proxy id for that subarchitecture.
   */
  const std::string &  planningVar2ProxyID(const std::string & _var, 
					   const std::string & _sa) const;

  /*
   * Given a plannign var returns the first related proxy id.
   */
  const std::string &  planningVar2ProxyID(const std::string & _var) const;

  
  virtual void start();

  virtual void configure(std::map<std::string,std::string> & _config);

  virtual bool
  handleAction(const std::string & _originalActionID,
	       const planning::autogen::Action & _action, 
	       const Planner::Command & _cmd);
  
  /**
  0 * Register an action with the planner.
   */
  template <class HandlerT>
  void registerAction(const std::string & _action, 
		      HandlerT * _handler) {

    assert(_handler != NULL);

    planning::autogen::ActionRegistration * act = new planning::autogen::ActionRegistration();
    act->m_component = CORBA::string_dup(getProcessIdentifier().c_str());
    act->m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
    act->m_action = CORBA::string_dup(_action.c_str());
    addToWorkingMemory( newDataID(), act);

    //TODO something smarter about existing etc.
    m_handlers[_action] = _handler;
  }

  /**
   * Register an action with the planner.
   */
  template <class HandlerT>
  void registerAction(const std::string & _action) {
    registerAction(_action, new HandlerT(*this));
  }


  void
  waitForAction(const std::string & _originalActionID, 
		const std::string & _newActionID, 
		const std::string & _actionSA,
		ActionHandler * _handler = NULL);

private:
  virtual void newAction(const cast::cdl::WorkingMemoryChange& _wmc);
  virtual void avmListAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  void updateVariableMappings(const motivation::idl::AddressVariableMappings & _avm);
  

  ///Receiver to get teh address of the avm list
  cast::WorkingMemoryChangeReceiver * m_avmReceiver;

  ///location of avm on wm
  std::string m_avmID;

  ///map of action handlers
  HandlerMap m_handlers;

  ///map containing avm
  /// planning var -> sourceID -> proxyID
  StringStringMapMap m_planningVar2proxyID;

  };



/**
 * Handler class for dispatching actions
 **/
class ActionHandler {
  
protected:
  ActionHandler(ActionDispatcher & _component) : 
    m_component(_component) {}
  virtual ~ActionHandler(){}
  
  ActionDispatcher & component() {
    return m_component;
  }

  Binding::BindingGraphHandler & bindingHandler() {
    return m_component.bindingHandler();
  }
  Binding::BindingGraphHandler & motiveHandler() {
    return m_component.motiveHandler();
  }

private:
  ActionDispatcher & m_component;

public:
   
  virtual bool
  handleAction(const std::string & _originalActionID,
	       const planning::autogen::Action & _action, 
	       const Planner::Command & _command) = 0;
    
  //called when an action completes
  virtual void
  actionComplete(planning::autogen::Action & _originalAction,
		 const Planner::Command & _originalCommand,
		 const planning::autogen::Action & _dispatchedAction, 
		 const cast::cdl::TriBool & _success) {}

};

/**
 * Always fail at action.
 */
class JustFailHandler : public ActionHandler {
public:
  JustFailHandler(ActionDispatcher & _component) : 
    ActionHandler(_component) {}
  virtual ~JustFailHandler(){}
  virtual bool handleAction(const std::string & _originalActionID,
			    const planning::autogen::Action & _action, 
			    const Planner::Command & _command) {
    return false;
  }
};



/**
 * Handler for acknowledgements
 */
class GoalAckHandler : public ActionHandler {
public:
  GoalAckHandler(ActionDispatcher & _component, 
		 const motivation::idl::MotiveAcknowledgement & _ack) : 
    ActionHandler(_component),
    m_ack(_ack){}

  virtual ~GoalAckHandler(){}
  virtual bool handleAction(const std::string & _originalActionID,
			    const planning::autogen::Action & _action, 
			    const Planner::Command & _command);
private:
  //the spatial rel to use
  motivation::idl::MotiveAcknowledgement m_ack;
};

/**
 * Handler for action sets
 */
class AnswerSetHandler : public ActionHandler {
public:
  AnswerSetHandler(ActionDispatcher & _component) : 
    ActionHandler(_component)
  {}

  virtual ~AnswerSetHandler(){}
  virtual bool handleAction(const std::string & _originalActionID,
			    const planning::autogen::Action & _action, 
			    const Planner::Command & _command);


  virtual void
  actionComplete(planning::autogen::Action & _originalAction,
		 const Planner::Command & _originalCommand,
		 const planning::autogen::Action & _dispatchedAction, 
		 const cast::cdl::TriBool & _success);

private:
};

 

/**
 * Handler for tell val actions
 */
class TellValHandler : public ActionHandler {


protected:

  virtual const std::string & getValType() = 0;
  
  /**
   * Default is a non instantiated pointer of the type from
   * getValType().
   */
  virtual void instantiateFeaturePointer(BindingData::FeaturePointer & _fp) {
    _fp.m_type = CORBA::string_dup(getValType().c_str());
    _fp.m_address = CORBA::string_dup("");
    _fp.m_immediateProxyID = CORBA::string_dup("");
  }


  /**
   * instantiate a feature pointer from a proxy for a target sa. This
   * prefers to use a value that is not from the target sa.
   **/
  virtual void instantiateFeaturePointer(BindingData::FeaturePointer & _fp,
					 const Binding::ProxyPtr & _proxy,
					 const std::string & _targetSA) = 0;


public:
  TellValHandler(ActionDispatcher & _component) : 
    ActionHandler(_component) {}
  virtual ~TellValHandler(){}
  virtual bool handleAction(const std::string & _originalActionID,
			    const planning::autogen::Action & _action, 
			    const Planner::Command & _command);

  virtual bool handleRequestAction(const std::string & _targetSA,
				   const std::string & _requestSA,			    
				   const std::string & _originalActionID,
				   const planning::autogen::Action & _action, 
				   const Planner::Command & _command);

  virtual bool handleAssertionAction(const std::string & _targetSA,
				     const std::string & _originalActionID,
				     const planning::autogen::Action & _action, 
				     const Planner::Command & _command);

  //called when an 
  virtual void
  actionComplete(planning::autogen::Action & _originalAction,
		 const Planner::Command & _originalCommand,
		 const planning::autogen::Action & _dispatchedAction, 
		 const cast::cdl::TriBool & _success);

 

};


/**
 * Handler for tell val actions, templated for a particular feature
 */
template <class FeatureT>
class TellFeatureTypeHandler : public TellValHandler {

public:
  TellFeatureTypeHandler(ActionDispatcher & _component) : 
    TellValHandler(_component) {}
  virtual ~TellFeatureTypeHandler(){}
 

protected:  
  virtual const std::string & getValType() {
    return cast::typeName<FeatureT>();
  }

  /**
   * Gets the feature from proxy in question
   */
  const FeatureT & getFeature(const Binding::ProxyPtr & _proxy,
			      const std::string & _targetSA) {

    const Binding::LBindingUnion & unionPtr(_proxy->bindingUnion());
    assert(unionPtr.hasFeature<FeatureT>());
    //return unionPtr.getFeature<FeatureT>();

    //get all the proxies for this union
    const std::set<std::string> & pids(unionPtr.proxyIDs());
    Binding::ProxySet proxies(bindingHandler().loadProxies(pids));
    
    //now look for the feature in a proxy that doesn't belong to the
    //targetSA
    
    std::string fallBackID("");
      for(Binding::ProxySet::const_iterator p = proxies.begin();
	  p != proxies.end(); ++p) {

	if(p->second->hasFeature<FeatureT>()) {
	  if(p->second->hasFeature<BindingFeatures::SourceID>()) {
	    const BindingFeatures::SourceID sid(p->second->getFeature<BindingFeatures::SourceID>());
	    std::string sourceID(sid.m_sourceID);
	    if(sourceID != _targetSA) {
	      component().log("TellFeatureTypeHandler: using feature from: %s", sourceID.c_str());
	      return p->second->getFeature<FeatureT>();
	    }
	    else {
	      //store for fallback
	      fallBackID = p->first;
	    }
	  }
	  //else its not safe to check, but keep for fall back
	  fallBackID = p->first;
	}

      }		
 
      //if we get here then use fall back
      assert(fallBackID != "");
      assert(proxies.find(fallBackID) != proxies.end());
      const Binding::ProxyPtr & fallBack(proxies.find(fallBackID)->second);
      return fallBack->getFeature<FeatureT>();      
  }

  /**
   * Instantiates the feature pointer with a new copy of the FeatureT
   * from the proxy
   */
  virtual void instantiateFeaturePointer(BindingData::FeaturePointer & _fp,
					 const Binding::ProxyPtr & _proxy,
					 const std::string & _targetSA) {

    //get feature
    const FeatureT & feature(getFeature(_proxy,_targetSA));
    //where it will go
    std::string id(component().newDataID());
    const std::string & sa(component().bindingSubarchID());    
    //and the copy itself
    FeatureT * featureCopy = new FeatureT(feature);

    //write this to wm
    component().addToWorkingMemory(id,sa, featureCopy, cast::cdl::BLOCKING);

    //now instantiate with details
    _fp.m_type = CORBA::string_dup(getValType().c_str());
    _fp.m_address = CORBA::string_dup(id.c_str());
    _fp.m_immediateProxyID = CORBA::string_dup("");
  }



};

/**
 * Handler for tell val actions
 */
class TellRelationLabelHandler : public TellValHandler {
public:
  TellRelationLabelHandler(ActionDispatcher & _component, 
			   const std::string & _label) : 
    TellValHandler(_component),
    m_label(_label) {}
  virtual ~TellRelationLabelHandler(){}
 
protected:  
  virtual const std::string & getValType() {
    return cast::typeName<BindingFeatures::RelationLabel>(); 
  }


  virtual void instantiateFeaturePointer(BindingData::FeaturePointer & _fp);

  //TODO fill this in for assertions
  virtual void instantiateFeaturePointer(BindingData::FeaturePointer & _fp,
					 const Binding::ProxyPtr & _proxy,
					 const std::string & _targetSA) {}


private:
  std::string m_label;
  std::string m_bsa;


};


 
#endif
