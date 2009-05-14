#ifndef MOTIVE_MANAGER_H_
#define MOTIVE_MANAGER_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <queue>
#include <motivation/idl/MotivationData.hh>

/**
 * A component that monitors the proxies of other subarchitectures
 * (specified by the --monitor a,b,c flag) to keep track of the
 * proxies that should be used to generate goals for the system. If
 * a consisten planning variable is required for a particular
 * subarchitecture, specify the subarch with the --variables flag.
 *
 * 
 */
class MotiveManager : public cast::ManagedProcess {
private:
  //typedef __gnu_cxx::hash_set<std::string> StringSet;
  //typedef StringMap< std::pair<std::std::string, std::string> >::map StringPairMap;
  //  typedef StringMap<StringStringMap>::map SSStringMap;

public:

  MotiveManager(const std::string &_id);
    
  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string> & _config);
  

protected:
  virtual void taskAdopted(const std::string &_taskID){}
  virtual void taskRejected(const std::string &_taskID){}
  virtual void runComponent();

private:

  class PlanningResultsReceiver : public cast::WorkingMemoryChangeReceiver {
  public:
    PlanningResultsReceiver(MotiveManager & _component) 
      : m_component(_component) {}
    void workingMemoryChanged(const cast::cdl::WorkingMemoryChange& _wmc) {
      boost::shared_ptr<const planning::autogen::PlanningProcessRequest> ppr(m_component.getWorkingMemoryEntry<planning::autogen::PlanningProcessRequest>(_wmc.m_address)->getData());
      if(m_component.planningRequestOverwritten(*ppr)) {
	m_component.m_resultsReceiver = NULL;
	m_component.deleteFromWorkingMemory(std::string(_wmc.m_address.m_id));
	m_component.removeChangeFilter(this, cast::cdl::DELETE_RECEIVER);
      }
    }
  private:
    MotiveManager & m_component;
  };

  friend class PlanningResultsReceiver;
  
  /**
   */
  bool planningRequestOverwritten(const planning::autogen::PlanningProcessRequest & _ppr);
  /**
   * Called when a new motive is added to wm.
   */
  void motiveAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  /**
   * Overwrite a motive, updating its status and success fields.
   */
  void updateMotive(const std::string & _motiveID,
		    const motivation::idl::Motive & _motive,
		    const motivation::idl::MotiveStatus & _status,
		    const cast::cdl::TriBool & _succeeded = cast::cdl::triIndeterminate);

  /**
   * Make stuff happen.
   */
  void processMotive(const std::string & _motiveID);

  /**
   * Motive is done
   */
  void motiveProcessingComplete(const cast::cdl::TriBool & _succeeded);




  //currently we just work through a queue of motives
  std::queue<std::string> m_motiveQueue;
  //the current motive we're working on
  std::string m_currentMotiveID;
  ///receiver for get planning results
  cast::WorkingMemoryChangeReceiver * m_resultsReceiver;
  
}; // MotiveManager

#endif //MOTIVE_MANAGER_H_

