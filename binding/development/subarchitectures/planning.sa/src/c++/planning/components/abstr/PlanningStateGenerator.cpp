/**
 * Planning State Generator -- equivalent of the abstract super class
 * in Java...
 *
 * @author Mohan
 * @date October 19 2007
 */

#include <iostream>
#include <cstdlib>
#include <planning/util/PlanningDataTranslator.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/architecture/WorkingMemoryChangeReceiver.hpp>

#include "PlanningStateGenerator.hpp"


// /**
//  * The function called to create a new instance of our component...
//  */
// extern "C" {
//   FrameworkProcess* newComponent( const std::string& _id ) {
//     return new PlanningStateGenerator(_id);
//   }
// }


/**
 * Constructor, because something has to exist because it can be
 * used -- remember to pass on the id to the super class...
 */
PlanningStateGenerator::PlanningStateGenerator( const std::string& _id ) : 
  WorkingMemoryAttachedComponent(_id),
  PrivilegedManagedProcess(_id) {}



/**
 * Destructor, where we clean up the mess we have created in this
 * class...
 */
PlanningStateGenerator::~PlanningStateGenerator() {
  cout << "PlanningStateGenerator::~PlanningStateGenerator()" << endl;

  // Remember to clear memory... :)
  m_subarchitectures.clear();
  m_translators.clear();
  m_taskMap.clear();
}




/**
 * Function to extend the _state (TemporaryPlanningState) by adding
 * the contents of _substate...
 */
void PlanningStateGenerator::extendPlanningState( TemporaryPlanningState& _state, 
						  const TemporaryPlanningState & _substate ) {
  // NOTE: I think the copy would work, but needs testing to make sure
  // I am not overrunning the vector -- if so, just replace with a
  // simple for loop...

  // Add the facts first, from the _substate to the end of factList of
  // _state...
//   for( size_t i = 0; i < _substate.m_factList.size(); ++i ) {
//     _state.m_factList.insert( _substate.m_factList.at(i) );
//   }
  
  _state.m_factList.insert(_substate.m_factList.begin(),_substate.m_factList.end());

//   copy( _substate.m_factList.begin(), _substate.m_factList.end(),
// 	_state.m_factList.end() ); 

  // Then add the objects to the end of the the objectList of
  // _state...
//   for( size_t i = 0; i < _substate.m_objectList.size(); ++i ) {
//     _state.m_objectList.insert( _substate.m_objectList.at(i) );
//   }
//   copy( _substate.m_objectList.begin(), _substate.m_objectList.end(),
// 	_state.m_objectList.end() );

  _state.m_objectList.insert(_substate.m_objectList.begin(),_substate.m_objectList.end());


}




/**
 * Function that generates the planning state -- based on the keys in
 * the data translator, the working memory entries in the
 * corresponding subarchitecture are accessed to get the planning
 * state, which is appended to the current planning state...
 */
PlanningStateLists PlanningStateGenerator::generateState() {
  TemporaryPlanningState state;
  log( "Starting state creation..." );
  
  // Set up the iterator and iterate through the valid keys...
  map<string, PlanningDataTranslatorBase*>::iterator iter;   
  for( iter = m_translators.begin(); iter != m_translators.end(); ++iter ) {
    // cout << "key: " << iter->first << ", value: " << iter->second << endl;
    // Get the corresponding subarchitecture id -- the 'value'
    // corresponding to the key in the subarchitectures map...
    string subarchitecture = m_subarchitectures[ iter->first ];
    log( "PSG:Looking at subarch... " + subarchitecture );

    // Also get the PlanningDataTranslator corresponding to the 'key'
    // currently being considered -- the 'value' in the translators
    // map...
    PlanningDataTranslatorBase* translator = m_translators[ iter->first ];
    log( "PSG:Looking at translator with key: " + iter->first );

    // Then get the working memory entries and hence the
    // TemporaryPlanningState, using the PlanningDataTranslator...
    TemporaryPlanningState substate = translator->toPlanningState( subarchitecture, 
								   iter->first, *this );
//     if( !substate.m_factList.empty() ) {
//       log( "PSG:Looking at state... " + substate.m_factList.at(0) );
//     }
//     else if( substate.m_factList.empty() && substate.m_objectList.empty() ) {
//       log( "PSG:Empty facts and objects lists..." );
//     }
	
    // Extend the planning state with the newly obtained state...
    if( substate.m_factList.size() > 0 || 
	substate.m_objectList.size() > 0 ) {
      extendPlanningState( state, substate );
//       log( "PSG:extended planning state..." );

//       for( size_t i = 0; i < state.m_factList.size(); ++i ) {
// 	std::cout << "PSG: state fact is: " << state.m_factList.at(i).name << "\n";
//       }

//       for( size_t i = 0; i < state.m_objectList.size(); ++i ) {
// 	std::cout << "PSG: state object is: " << state.m_objectList.at(i).name << " " << state.m_objectList.at(i).type << "\n";
//       }
    }
  }
  
  // Now convert the state to the original PlanningState and return
  // it...
  return state.toPlanningState();
}









/**
 * Function takes the input id, which maps to a working memory
 * address, and gets a working memory entry, thereby getting the
 * task to execute, which is written on the working memory...
 */
void PlanningStateGenerator::taskAdopted( const std::string& _taskID ) {
  // Get the WMA address corresponding to the task...
  WorkingMemoryAddress requestAddr = m_taskMap[ _taskID ];
  log("PSG.taskadopted: " + _taskID );
  
  // Get the working memory entry and hence a planningstaterequest
  // pointer...
  shared_ptr< const CASTTypedData<PlanningStateRequest> > psr_ptr = 
    getWorkingMemoryEntry<PlanningStateRequest>( requestAddr );
  PlanningStateRequest psr = *( psr_ptr->getData() );

  // Check if the state is empty -- a different treatment is required
  // if the state has already be written to...
  if( psr.m_state.m_facts.length() != 0 ||
      psr.m_state.m_objects.length() != 0 ) {
    // Let the Human know that the approach leads to complete
    // failure...
    log("PSG.taskadopted: states or objects not empty..." );
    taskComplete( _taskID, TaskOutcome(PROCESSING_COMPLETE_FAILURE) ); // PROCESSING_COMPLETE_FAILURE;
    return;
  }

  // On the other hand if the state is empty...
  try {
    log("PSG.taskadopted: trying to generate state..." );
    // We generate the state using the derived class...
    psr.m_state = generateState();
    
    log("PSG.taskadopted: generated state, writing to WM..." + (string)requestAddr.m_subarchitecture );
    // Overwrite the working memory's existing request...
    overwriteWorkingMemory( requestAddr, new PlanningStateRequest( psr ) );

    // Signal task completion...
    taskComplete( _taskID, TaskOutcome(PROCESSING_COMPLETE_SUCCESS) ); // PROCESSING_COMPLETE_SUCCESS;

  }
  catch( SubarchitectureProcessException ex ) {
    // Let everybody know of the failure...
    taskComplete( _taskID, TaskOutcome(PROCESSING_COMPLETE_FAILURE) ); // PROCESSING_COMPLETE_FAILURE;
  }
}




/**
 * Removes a certain task from the task map, specified by means of the
 * key (in the key-value pair)...
 */
void PlanningStateGenerator::taskRejected( const std::string& _taskID ) {
  log("PSG.taskrejected: " + _taskID );
  m_taskMap.erase( _taskID );
}



/**
 * Function that triggers based on local state requests...
 */
void PlanningStateGenerator::start() {
  PrivilegedManagedProcess::start();

  // Listen for state requests that occur locally...
  try {
    log("PSG.start: registered a change filter..." );

    
    addChangeFilter(createLocalTypeFilter<PlanningStateRequest>(WorkingMemoryOperation(ADD)),
		    new MemberFunctionChangeReceiver<PlanningStateGenerator>(this, &PlanningStateGenerator::planStateChanged));
  }
  catch( SubarchitectureProcessException ex ) {
//     std::cout << "\n Could not add change filter in the PlanningState Generator ...\n";
//     exit(1);
  }
}




/**
 * Function is called as a receiver for the change filter in the
 * 'start' function -- basically handles the change by adding a new
 * entry into the map that stores key-value pairs of
 * strings-workingmemory address...
 */
void PlanningStateGenerator::planStateChanged( const cdl::WorkingMemoryChange& _wmc ) {
  try {
    // log("PSG.planstatechanged: insert into taskmap..." );
    string taskID = newTaskID();
    m_taskMap.insert( pair<string,WorkingMemoryAddress>(taskID, _wmc.m_address) );
    
    // Propose the information processing task (part of
    // ManagedProcess) to tackle next...
    string taskName = string("fulfil-state-request-") + 
      string(_wmc.m_address.m_id);
    proposeInformationProcessingTask( taskID, taskName );
  }
  catch( SubarchitectureProcessException ex ) {
//     std::cout << "\n Plan State Changed failed in PlanningStateGenerator... \n";
//     exit( 1 );
  }
}




/**
 * Default runComponent function for the CASTComponent that does
 * nothing here...
 */
void PlanningStateGenerator::runComponent() {

}
