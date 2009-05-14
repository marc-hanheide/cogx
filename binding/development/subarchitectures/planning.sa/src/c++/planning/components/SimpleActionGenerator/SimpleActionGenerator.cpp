/**
 * Simple Action Generator...
 *
 * @author Mohan
 * @date October 19 2007
 */

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <string>
#include <ctime>
#include "SimpleActionGenerator.hpp"
#include "planning/util/PlanningUtils.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace boost;

/**
 * The function called to create a new instance of our component...
 */
extern "C" {
  FrameworkProcess* newComponent( const std::string& _id ) {
    return new SimpleActionGenerator(_id);
  }
}



/**
 * Constructor...
 */
SimpleActionGenerator::SimpleActionGenerator( const std::string _id ) :
  WorkingMemoryAttachedComponent( _id ),
  ManagedProcess( _id ) {
}


/**
 * Destructor...
 */
SimpleActionGenerator::~SimpleActionGenerator() {}



/**
 * Lets get the actions rolling...
 */
void SimpleActionGenerator::start() {
  // Trigger the base class as well...
  ManagedProcess::start();
  // Add a change filter to the setup...
  try {
    addChangeFilter( createLocalTypeFilter<Action>(WorkingMemoryOperation(ADD)), 
		     this );
  }
  catch( const SubarchitectureProcessException & ex ) {
    cout << "\n Starting trouble in SimpleActionGenerator...\n";
    std::abort( );
  }  

}



/**
 * Function that responds to working memory change...
 */
void SimpleActionGenerator::workingMemoryChanged( const WorkingMemoryChange& _wmc ) {
  try {
    // sleepProcess(1000);
    
    // Get the action to perform...
    shared_ptr< const CASTTypedData<Action> > action_ptr = 
      getWorkingMemoryEntry<Action>( _wmc.m_address );
    Action action = *( action_ptr->getData() );

    srand ( time(NULL) );

    double r((double)rand() / (double)RAND_MAX);
    
    if (r >= m_failureChance) {
      log( "Performing action now -- start praying..." );
      performAction(action);
    }
    else {
      log("not performing action");
    }


    // Randomly decide to perform or not perform the action...
//     if( ( rand() % 10 ) >= 5 ) {
//       log( "Performing action now -- start praying..." );
//       performAction( action );
//     }
//     else {
//       log( "Not performing any action now..." );
//     }
    
    // Manually fix the status bits of the action variable...
    action.m_status = PlanningStatus( COMPLETE );
    action.m_succeeded = TriBool( triTrue );
    
    // Overwrite the working memory entries...
    overwriteWorkingMemory( string(_wmc.m_address.m_id), 
			    new Action( action ) , 
			    BLOCKING);
  }
  catch( SubarchitectureProcessException ex ) {
    cout <<"\n Change detection in SimpleActionGenerator failed...\n";
    std::abort();
  }
}



/**
 * Function that simulates the action performance...
 */
void SimpleActionGenerator::performAction( Action _action ) {
  try {
    // Get the current action step to be executed...
    shared_ptr< const CASTTypedData<Command> > step_ptr = 
      getWorkingMemoryEntry<Command>( _action.m_action.m_address);
    Command step = *( step_ptr->getData() );
    log("Performing step: %s", string(step.mapl_action.name).c_str());

    // Get the fact additions and deletions corresponding to the
    // action step...
    vector<Fact> addVector;
    vector<Fact> delVector;
    getAdditions( step, addVector );
    getDeletions( step, delVector );

    // Run through the additions and make corresponding changes to the
    // working memory...
    for( size_t i = 0; i < addVector.size(); ++i ) {
      log("Adding fact ");
      addToWorkingMemory( newDataID(), 
			  new Fact( addVector.at(i) ) );
    }

    // Clear out the vectors of strings used...
    addVector.clear();

    // Then make changes corresponding to the deletions in the vector
    // populated earlier -- First get the facts in the current working
    // memory...
    vector< shared_ptr< const CASTData<Planner::Fact> > > facts;
    getWorkingMemoryEntries<Planner::Fact>(0, facts);

    // If no facts exist, do nothing else...
    if( facts.empty() ) {
      cout << "\n No facts to delete...\n";
      facts.clear(); // Stupid, but just in case...
      return;
    }
    
    // If the delete vector is empty, there is not much to do...
    if( delVector.empty() ) {
      cout << "\n No facts in delete vector...\n";
      // Nothing to delete -- so get out of here, after clearing up
      // the allocated memory of course...
      facts.clear();
      delVector.clear();
      return;
    }

    // For each element in the delete list, remove the corresponding
    // entry from the working memory if it exists in the vector of
    // facts...
    for( size_t i = 0; i < facts.size(); ++i ) {
      // Get the current fact in the list of known facts...
      Fact fact = ( *( (facts.at(i))->getData() ) );
     

      // See if this fact matches any of the facts in the delete
      // vector...
      for( size_t j = 0;  j < delVector.size(); ++j ) {
	const Fact& target(delVector.at(j));
	if( target ==  fact ) {
	  log("Removing fact from WM: %s",(facts.at(i))->getID().c_str() );
	  deleteFromWorkingMemory( (facts.at(i))->getID() );
	}
      }
    }

    // Clear up the facts stream...
    facts.clear();

    // Clear out the vectors of strings used...
    delVector.clear();
    
  }
  catch( const SubarchitectureProcessException & ex ) {
    cout << "\n Could not performAction...\n";
    std::abort();
  }
}




/**
 * Function that simulates the deletion of steps, as a result of
 * actions...
 */
void SimpleActionGenerator::getDeletions( const Command & _step, 
					  vector<Fact>& delVector ) {
  

  if (strcmp(_step.mapl_action.name,"pick_up") == 0) {

    delVector.push_back( Fact() );
    delVector.back().name = CORBA::string_dup( "pos" );
    delVector.back().arguments.length(1);
    delVector.back().arguments[0] = CORBA::string_dup( _step.mapl_action.args[0] );
    delVector.back().value = CORBA::string_dup( _step.mapl_action.args[1] );

  }
  else if (strcmp(_step.mapl_action.name,"place") == 0) {

    delVector.push_back( Fact() );
    delVector.back().name = CORBA::string_dup( "pos" );
    delVector.back().arguments.length(1);
    delVector.back().arguments[0] = CORBA::string_dup( _step.mapl_action.args[0] );
    delVector.back().value = CORBA::string_dup( "mrchips" );

  }
  else if (strcmp(_step.mapl_action.name,"move") == 0) {
    println("return ing deletions for move... probably doesn't working properly!");

    delVector.push_back( Fact() );
    delVector.back().name = CORBA::string_dup( "pos" );
    delVector.back().arguments.length(1);
    delVector.back().arguments[0] = CORBA::string_dup( _step.mapl_action.args[1] );
    delVector.back().value = CORBA::string_dup( _step.mapl_action.args[3] );

  }


}


/**
 * Function that simulates the addition of steps, as a result of
 * actions...
 */
void SimpleActionGenerator::getAdditions( const Command & _step, 
					  vector<Fact>& addVector ) {

  if (strcmp(_step.mapl_action.name,"pick_up") == 0) {

    addVector.push_back( Fact() );
    addVector.back().name = CORBA::string_dup( "pos" );
    addVector.back().arguments.length(1);
    addVector.back().arguments[0] = CORBA::string_dup( _step.mapl_action.args[0] );
    addVector.back().value = CORBA::string_dup( "mrchips" );

  }
  else if (strcmp(_step.mapl_action.name,"place") == 0
	   || strcmp(_step.mapl_action.name,"move") == 0) {

    addVector.push_back( Fact() );
    addVector.back().name = CORBA::string_dup( "pos" );
    addVector.back().arguments.length(1);
    addVector.back().arguments[0] = CORBA::string_dup( _step.mapl_action.args[0] );
    addVector.back().value = CORBA::string_dup( _step.mapl_action.args[1] );

  }


}




/**
 * Dummy function not currently in use...
 */
void SimpleActionGenerator::taskAdopted( const std::string& _taskID ) {

}



/**
 * Dummy function not currently in use...
 */
void SimpleActionGenerator::taskRejected( const std::string& _taskID ) {

}
  


/**
 * Default runComponent function for the CASTComponent that does
 * nothing here...
 */
void SimpleActionGenerator::runComponent() {
  // For now, some simple actions being registered here for the
  // purpose of playing with the visual planning operators in
  // simulation...
  ActionRegistration siftOp;
  siftOp.m_component = CORBA::string_dup( getProcessIdentifier().c_str() );
  siftOp.m_subarchitecture = CORBA::string_dup( m_subarchitectureID.c_str() );
  siftOp.m_action = CORBA::string_dup( string("pick_up").c_str() );
  addToWorkingMemory( newDataID(), new ActionRegistration( siftOp ) );

  ActionRegistration colorOp;
  colorOp.m_component = CORBA::string_dup( getProcessIdentifier().c_str() );
  colorOp.m_subarchitecture = CORBA::string_dup( m_subarchitectureID.c_str() );
  colorOp.m_action = CORBA::string_dup( string("place").c_str() );
  addToWorkingMemory( newDataID(), new ActionRegistration( colorOp ) );

  ActionRegistration shapeOp;
  shapeOp.m_component = CORBA::string_dup( getProcessIdentifier().c_str() );
  shapeOp.m_subarchitecture = CORBA::string_dup( m_subarchitectureID.c_str() );
  shapeOp.m_action = CORBA::string_dup( string("move").c_str() );
  addToWorkingMemory( newDataID(), new ActionRegistration( shapeOp ) );  


}


void SimpleActionGenerator::configure(std::map<std::string,std::string> & _config) {
  ManagedProcess::configure(_config);

  std::map<std::string,std::string>::const_iterator i =  _config.find("--failure");
  if(i != _config.end()) {
    m_failureChance = atof(i->second.c_str());
  }
  else {
    m_failureChance = 0.5;
  }

}
