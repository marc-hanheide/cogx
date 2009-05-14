#ifndef PLAYMATE_ACTION_DISPATCHER_H_
#define PLAYMATE_ACTION_DISPATCHER_H_

#include <motivation/components/ActionDispatcher/ActionDispatcher.hpp>
#include <spatial/idl/SpatialData.hh>
#include <binding/idl/BindingQueries.hh>
#include <planning/idl/PlanningData.hh>




class PlaymateActionDispatcher : public ActionDispatcher {
public:
  
  PlaymateActionDispatcher(const std::string & _id);
  virtual ~PlaymateActionDispatcher() {}

  virtual void taskAdopted(const std::string &_taskID) {}
  virtual void taskRejected(const std::string &_taskID) {}
  virtual void runComponent();

};




/**
 * Handler for reposition actions
 */
class RepositionHandler : public ActionHandler {
public:
  RepositionHandler(ActionDispatcher & _component, 
		    const spatial::autogen::SpatialRelationshipType & _rel) : 
    ActionHandler(_component),
    m_rel(_rel){}

  virtual ~RepositionHandler(){}
  virtual bool handleAction(const std::string & _originalActionID,
			    const planning::autogen::Action & _action, 
			    const Planner::Command & _command);
private:
  //the spatial rel to use
  spatial::autogen::SpatialRelationshipType m_rel;
};


/**
 * Handler for manipulation actions
 */
class ManipulationHandler : public ActionHandler {
public:
  ManipulationHandler(ActionDispatcher & _component) : 
    ActionHandler(_component) {}
  virtual ~ManipulationHandler(){}
  virtual bool handleAction(const std::string & _originalActionID,
			    const planning::autogen::Action & _action, 
			    const Planner::Command & _command);
};



 
#endif
