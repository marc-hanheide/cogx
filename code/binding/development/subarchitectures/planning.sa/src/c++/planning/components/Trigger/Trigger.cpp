/**
 * Simple Trigger to get everything else started...
 *
 * @author Mohan
 * @date October 19 2007
 */

#include <iostream>
#include <cstdlib>

#include <cast/core/CASTData.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include "Trigger.hpp"


/**
 * The function called to create a new instance of our component...
 */
extern "C" {
  FrameworkProcess* newComponent( const std::string& _id ) {
    return new Trigger(_id);
  }
}


/**
 * The constructor...
 */
Trigger::Trigger( const std::string _id ) :
  WorkingMemoryAttachedComponent( _id ),
  PrivilegedManagedProcess( _id ) {

  // Reset the scne change processing flag...
  //m_sceneChangeProcessed = false;

}


/**
 * The destructor...
 */
Trigger::~Trigger() {
  // Clean up the vectors -- we do not want memory leaks...:)
  m_facts.clear();
  m_objects.clear();
}



/**
 * Just a dummy start function to register a change filter...
 */
void Trigger::start() {
  PrivilegedManagedProcess::start();
}



/**
 * Function that runs the component, i.e. builds the state, writes it
 * into the working memory and requests a planning process...
 */
void Trigger::runComponent() {
  try {
    // Build the state from the facts and objects -- TAKE THIS OUT
    // WHEN NOT IN SIMULATION...
    buildState();
    // Write the state to working memory -- TAKE THIS OUT WHEN NOT IN
    // SIMULATION...
    writeState();
    
    // Set the goal state...

    // string goal1( "(and (exists (?vr - visRegion) (and ( colorProp-val ?vr blue ) ( shapeProp-val ?vr circular ) ) ) )" );
    //string goal1("(and (exists (?vr - visRegion) (and ( containsColor ?vr blue ) ( containsShape ?vr circular ) ) ) )" );
    string goal1( "(and (pos obj1 : dwp_0))" );
    
    // string goal1( "(pos obj1 dwp_0)" );
    log( "Trying for goal state: %s",goal1.c_str() );
    
    // Well then request the planning process to attempt to get to the
    // goal state, and hope and pray that everything works fine...
    requestPlanningProcess( goal1 );
//     sleep( 10 );
    cout << "...\n ...\n ...\n ...\n";
    // log( "OK, I am back after a short nap, give me another goal to achieve...");
    
  }
  catch( const SubarchitectureProcessException & ex ) {
    cout << "\n Problem in running component in Trigger...\n";
  }

}

void Trigger::addObjectDeclaration(const std::string& _name,
				   const std::string& _type) {

  m_objects.push_back( ObjectDeclaration() );
  m_objects.back().name = CORBA::string_dup( _name.c_str() );
  m_objects.back().type = CORBA::string_dup( _type.c_str() );
}

void Trigger::addFact(const std::string& _name,
		      const std::string _args[],
		      const unsigned int & _arglen,
		      const std::string& _value) {

  m_facts.push_back( Fact() );
  m_facts.back().name = CORBA::string_dup( _name.c_str() );

  m_facts.back().arguments.length(_arglen);
  for (unsigned int i = 0; i < _arglen; ++i) {
    m_facts.back().arguments[i] = CORBA::string_dup( _args[i].c_str() );
  }

  m_facts.back().value = CORBA::string_dup( _value.c_str() );

}


/**
 * Self-explanatory -- build the notion of state from the facts and
 * objects of the domain -- FOR THE VISUAL PLANNING DOMAIN...
 */
void Trigger::buildState() {
  // DUMMY THAT NEEDS TO BE GUTTED OUT BEFORE ACTUALLY RUNNING STUFF
  // ON THE ROBOT...  


  addObjectDeclaration("obj1", "movable");
  addObjectDeclaration("obj2", "movable");
  addObjectDeclaration("obj3", "movable");
  addObjectDeclaration("wpd", "waypoint");
  addObjectDeclaration("wpa", "waypoint");
  addObjectDeclaration("wp9", "waypoint");
  addObjectDeclaration("dwp_0", "waypoint");
  addObjectDeclaration("dwp_1", "waypoint");
  addObjectDeclaration("dwp_2", "waypoint");
  addObjectDeclaration("dwp_3", "waypoint");
  addObjectDeclaration("dwp_4", "waypoint");
  addObjectDeclaration("dwp_5", "waypoint");
  addObjectDeclaration("dwp_6", "waypoint");
  addObjectDeclaration("dwp_7", "waypoint");
  addObjectDeclaration("dwp_8", "waypoint");
  addObjectDeclaration("dwp_9", "waypoint");
  addObjectDeclaration("dwp_10", "waypoint");
  addObjectDeclaration("dwp_11", "waypoint");
  addObjectDeclaration(PLAYMATE_AGENT_NAME, "robot");
  addObjectDeclaration(PLAYMATE_AGENT_NAME,
				      "planning_agent");
  // Add the facts in the world...

  {string args[] = {"obj1"};
  addFact("colour", args, 1, "blue");}
  {string args[] = {"obj2"};
  addFact("colour", args, 1, "blue");}
  {string args[] = {"obj3"};
  addFact("colour", args, 1, "red");}
  {string args[] = {"obj1"};
  addFact("pos", args, 1, "wpa");}
  {string args[] = {"obj2"};
  addFact("pos", args, 1, "wp9");}
  {string args[] = {"obj3"};
  addFact("pos", args, 1, "wpd");}
  
  {string args[] = {"dwp_0", "wpd"};
  addFact("wp_left_of", args, 2);}
  {string args[] = {"dwp_1", "wpd"};
  addFact("wp_left_of", args, 2);}
  {string args[] = {"dwp_2", "wpa"};
  addFact("wp_left_of", args, 2);}
  {string args[] = {"dwp_3", "wpa"};
  addFact("wp_left_of", args, 2);}
  {string args[] = {"dwp_4", "wp9"};
  addFact("wp_left_of", args, 2);}
  {string args[] = {"dwp_5", "wp9"};
  addFact("wp_left_of", args, 2);}
  {string args[] = {"dwp_6", "wpd"};
  addFact("wp_right_of", args, 2);}
  {string args[] = {"dwp_7", "wpd"};
  addFact("wp_right_of", args, 2);}
  {string args[] = {"dwp_8", "wpa"};
  addFact("wp_right_of", args, 2);}
  {string args[] = {"dwp_9", "wpa"};
  addFact("wp_right_of", args, 2);}
  {string args[] = {"dwp_10", "wp9"};
  addFact("wp_right_of", args, 2);}
  {string args[] = {"dwp_11", "wp9"};
  addFact("wp_right_of", args, 2);}
  
}




/**
 * Function to add the built state to the working memory...
 */
void Trigger::writeState() {
  // First write out the objects...
  for( size_t i = 0; i < m_objects.size(); ++i ) {
    log("Adding object: %s %s",string(m_objects.at(i).name).c_str(),string(m_objects.at(i).type).c_str() );
    addToWorkingMemory( newDataID(), "action.sa", new ObjectDeclaration( m_objects.at(i) ) , BLOCKING);
  }

  // Then write out the facts...
  for( size_t i = 0; i < m_facts.size(); ++i ) {
    log("Adding fact: %s", string(m_facts.at(i).name).c_str() );
    addToWorkingMemory( newDataID(), "action.sa", new Fact( m_facts.at(i) ) , BLOCKING);
  }

}




/**
 * Just a precautionary funtion to ensure that all objects are written
 * out to the working memory...
 */
void Trigger::flush() {
//   if( m_inputToLocalWorkingMemory != NULL ) {
//     m_inputToLocalWorkingMemory.flush();
//   }
//   else {
//     m_inputToRemoteWorkingMemory.flush();
//   }
}




/**
 * Function that requests a planning process for a given goal,
 * subarchitecture and domain...
 */
void Trigger::requestPlanningProcess( const std::string& _goal ) {
  // String of subarchitecture IDs to be accessed...
  // string subArch = { m_subarchitectureID };
  try {
//     cast::cdl::SubarchitectureIDList subarchIDs;
//     subarchIDs.length(1);
//     subarchIDs[0] = m_subarchitectureID;
    //string tempV = *m_subarchitectureID;
    cout << "TRG: requesting for goal: " << _goal << endl;
    PlanningProcessRequest ppr;
    ppr.m_maplGoal = CORBA::string_dup( _goal.c_str() );
    ppr.m_domainName = CORBA::string_dup( PLAYMATE_DOMAIN_NAME.c_str() );
    ppr.m_domainFile = CORBA::string_dup( PLAYMATE_DOMAIN_FILE.c_str() );
    ppr.m_agent = CORBA::string_dup( PLAYMATE_AGENT_NAME.c_str() );    
    ppr.m_contributors.length(1);
    ppr.m_contributors[0] = CORBA::string_dup( "action.sa" );
    ppr.m_status = PlanningStatus( PROPOSED );
    ppr.m_succeeded = TriBool( triIndeterminate );


    string id(newDataID());    
    addChangeFilter(createIDFilter(id,OVERWRITE), this);
    addToWorkingMemory( id,  
			new PlanningProcessRequest( ppr ),
			cdl::BLOCKING);
  }
  catch( SubarchitectureProcessException ex ) {
    cout << "\n Request to the planning process did not go through...\n";
    // exit( EXIT_FAILURE );
  }
}




/**
 * Function that is not exactly required here...
 */
void Trigger::taskAdopted( const std::string& _taskID ) {

}



/**
 * Function that is not exactly required here...
 */
void Trigger::taskRejected( const std::string& _taskID ) {

}



void Trigger::workingMemoryChanged(const WorkingMemoryChange & _wmc) {
  
  try {
    
    shared_ptr< const CASTData<PlanningProcessRequest> > pprData 
      = getWorkingMemoryEntry<PlanningProcessRequest>(_wmc.m_address);
    shared_ptr<const PlanningProcessRequest> ppr = pprData->getData();
    
    if (ppr->m_status == planning::autogen::COMPLETE) {
      println("planning complete!: %d", ppr->m_succeeded);
      
      sleepProcess(2000);
      
      //for testing purposes
      if(ppr->m_succeeded == cdl::triTrue) {
	std::exit(testing::CAST_TEST_PASS);
      }
      else {
	std::exit(testing::CAST_TEST_FAIL);
      }
    }
  }
  catch (const SubarchitectureProcessException & e) {
    println(e.what());
    abort();
  }

}

