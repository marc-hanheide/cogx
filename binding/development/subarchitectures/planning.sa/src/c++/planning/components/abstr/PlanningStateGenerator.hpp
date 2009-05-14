/**
 * Here begins the adventure...Oct 19, 2007...
 */

/**
 * Planning State Generator -- equivalent of the abstract super class
 * in Java...
 *
 * @author Mohan
 * @date October 19 2007
 */

#ifndef PLANNING_STATE_GENERATOR_H_
#define PLANNING_STATE_GENERATOR_H_

#include <vector>
#include <map> 
#include <algorithm>
#include <string>

#include <cast/architecture/ManagedProcess.hpp>
#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include <cast/architecture/SubarchitectureProcessException.hpp>
#include <cast/architecture/WorkingMemoryReaderProcess.hpp>
#include <cast/architecture/WorkingMemoryChangeReceiver.hpp>

#include <cast/cdl/CAST.hh>
#include <cast/cdl/guitypes.hh>
#include <cast/core/CASTData.hpp>

#include <planning/idl/PlanningData.hh>
#include <planning/util/TemporaryPlanningState.hpp>

using namespace std;
using namespace Planner; 
using namespace planning::autogen; 
using namespace boost;
using namespace cast;
using namespace cast::cdl;


// Fwd declaration --- SORRY ABOUT THE MESS...:(
class PlanningDataTranslatorBase;
template <class T> class PlanningDataTranslator;

/**
 * This is the base class for components that can generate logical
 * descriptions of the current state from the contents of working
 * memory. When an instance of
 * PlanningOntology.PLANNING_STATE_REQUEST_TYPE is added to this
 * component's SA-local working memory, it reads in this object and
 * generates a state to fulfil the request...
 * 
 * @author Mohan
 */

class PlanningStateGenerator : public PrivilegedManagedProcess {
 public:
  /**
   * The standard constructors and destructors...
   */
  PlanningStateGenerator(const std::string& _id);
  ~PlanningStateGenerator();

  /**
   * Function that triggers based on local state requests...
   */
  void start();


  /**
   * Function is called as a receiver for the change filter in the
   * 'start' function -- basicallyy handles the change by adding a new
   * entry into the map that stores key-value pairs of
   * strings-workingmemory address...
   */
  void planStateChanged(const cdl::WorkingMemoryChange& _wmc);

 protected:

  /**
   * Function that generates the planning state -- based on the keys
   * in the data translator, the working memory entries in the
   * corresponding subarchitecture are accessed to get the planning
   * state, which is appended to the current planning state...
   */
  PlanningStateLists generateState();
  
  /**
   * Function registers an object to be used to translate a state
   * entry into a planning state...
   */
  template <class Type>
  void registerPlanStateMapping( PlanningDataTranslatorBase* _translator ) {
    registerPlanStateMapping<Type>( m_subarchitectureID, _translator );
  }

  /**
   * Overloaded version of function above -- registers an object to be
   * used to translate a state entry into a planning state...
   */
  template <class Type>
  void registerPlanStateMapping( const std::string &_subarch, 
				 PlanningDataTranslatorBase* _translator ) {
    string type(typeName<Type>());
    // Insert the key-value pair into the translators map...
    m_translators.insert( pair<string, PlanningDataTranslatorBase*>( type, _translator ) );
    // Insert the key-value pair into the subarchitectures map...
    m_subarchitectures.insert( pair<string, string>( type, _subarch ) );
  }

  /**
   * Function takes the input id, which maps to a working memory
   * address, and gets a working memory entry, thereby getting the
   * task to execute, which is written on the working memory...
   */
  void taskAdopted( const std::string& _taskID );

  /**
   * Removes a certain task from the task map, specified by means of
   * the key (in the key-value pair)...
   */
  void taskRejected( const std::string& _taskID );

  /**
   * Default runComponent function for the CASTComponent that does
   * nothing here...
   */
  void runComponent();

 private:
  /**
   * Function to extend the _state (TemporaryPlanningState) by adding
   * the contents of _substate...
   */
  void extendPlanningState( TemporaryPlanningState & _state, 
			    const TemporaryPlanningState & _substate );

  // Variables to be used instead of the hashtables in Java, i.e. to
  // deal with key-value pairs...
  map<string, string> m_subarchitectures;
  map<string, PlanningDataTranslatorBase*> m_translators;
  map<string, WorkingMemoryAddress> m_taskMap;

};





#endif
