#ifndef EXPLORER_ACTION_DISPATCHER_H_
#define EXPLORER_ACTION_DISPATCHER_H_

#include <motivation/components/ActionDispatcher/ActionDispatcher.hpp>



class ExplorerActionDispatcher : public ActionDispatcher {
public:
  
  ExplorerActionDispatcher(const std::string & _id);
  virtual ~ExplorerActionDispatcher() {}

  virtual void taskAdopted(const std::string &_taskID) {}
  virtual void taskRejected(const std::string &_taskID) {}
  virtual void runComponent();

};

/**
 * Handler for manipulation actions
 */
class NavCommandHandler : public ActionHandler {
public:
  NavCommandHandler(ActionDispatcher & _component) : 
    ActionHandler(_component) {}
  virtual ~NavCommandHandler(){}
  virtual bool handleAction(const std::string & _originalActionID,
			    const planning::autogen::Action & _action, 
			    const Planner::Command & _command);
};




 
#endif
