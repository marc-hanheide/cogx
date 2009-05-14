/**
 * Dummy manipulator.
 */

#ifndef PRETEND_MANIPULATOR_H_
#define PRETEND_MANIPULATOR_H_

#include <planning/idl/PlanningData.hh>

#include <cast/architecture/PrivilegedManagedProcess.hpp>

/**
 * Class that simulates a simple action generator, i.e. the
 * performance of actions and the changes in working memory...
 */
class PretendManipulator : 
  public cast::PrivilegedManagedProcess, 
  public cast::WorkingMemoryChangeReceiver {

public:
  /**
   * Constructor and destructor...
   */
  PretendManipulator(const std::string _id);
  ~PretendManipulator();
  
  /**
   * Lets get the actions rolling...
   */
  void start();

  /**
   * Function that responds to working memory change...
   */
  void workingMemoryChanged( const cast::cdl::WorkingMemoryChange& _wmc );


protected:
  /**
   * Dummy function not currently in use...
   */
  void taskAdopted( const std::string& _taskID );

  /**
   * Dummy function not currently in use...
   */
  void taskRejected( const std::string& _taskID );

  /**
   * Default runComponent function for the CASTComponent that does
   * nothing here...
   */
  void runComponent();

  virtual void configure(std::map<std::string,std::string> & _config);

private:
  /**
   * Function that simulates the action performance...
   */
  void performAction( const planning::autogen::Action & _action );

  void sceneChangedAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  double m_failureChance;

  
  ///Receiver to get the address of the SceneChanged struct
  cast::WorkingMemoryChangeReceiver * m_scReceiver;

  ///the address of the SceneChanged struct
  cast::cdl::WorkingMemoryAddress m_scAddress;

};





#endif
