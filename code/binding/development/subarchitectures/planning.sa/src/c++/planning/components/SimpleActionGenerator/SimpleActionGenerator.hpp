/**
 * Simple Action Generator...
 *
 * @author Mohan
 * @date October 19 2007
 */

#ifndef SIMPLE_ACTION_GENERATOR_H_
#define SIMPLE_ACTION_GENERATOR_H_

#include <vector>
#include <map> 
#include <algorithm>

#include <planning/idl/PlanningData.hh>

#include <cast/architecture/WorkingMemoryReaderProcess.hpp>
#include <cast/architecture/WorkingMemoryChangeReceiver.hpp>
#include <cast/architecture/ManagedProcess.hpp>
#include <cast/architecture/SubarchitectureProcessException.hpp>

#include <cast/cdl/CAST.hh>
#include <cast/cdl/guitypes.hh>
#include <cast/core/CASTData.hpp>

// Unfortunately the namespaces have to be set in this crude manner...
using namespace std; 
using namespace Planner; 
using namespace planning::autogen; 
using namespace cast;
using namespace cast::cdl;


/**
 * Class that simulates a simple action generator, i.e. the
 * performance of actions and the changes in working memory...
 */
class SimpleActionGenerator : 
  public ManagedProcess, 
  public WorkingMemoryChangeReceiver {
 public:
  /**
   * Constructor and destructor...
   */
  SimpleActionGenerator( const std::string _id );
  ~SimpleActionGenerator();

  /**
   * Lets get the actions rolling...
   */
  void start();

  /**
   * Function that responds to working memory change...
   */
  void workingMemoryChanged( const WorkingMemoryChange& _wmc );


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
  void performAction( Action _action );

  /**
   * Function that simulates the addition and deletion of steps, as a
   * result of actions...
   */
  void getDeletions( const Command &_step, vector<Fact>& delVector );
  void getAdditions( const Command &_step, vector<Fact>& addVector );

  double m_failureChance;

};





#endif
