#include "PretendManipulator.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <vision/idl/Vision.hh>
#include <manipulation/idl/Manipulation.hh>

using namespace std;
using namespace boost;
using namespace cast;
using namespace cast::cdl;
using namespace planning::autogen;
using namespace Vision;
using namespace Manipulation;



/**
 * Constructor...
 */
PretendManipulator::PretendManipulator( const std::string _id ) :
  cast::WorkingMemoryAttachedComponent( _id ),
  cast::PrivilegedManagedProcess( _id ),
  m_scReceiver(NULL) {
}


/**
 * Destructor...
 */
PretendManipulator::~PretendManipulator() {}



/**
 * Lets get the actions rolling...
 */
void PretendManipulator::start() {
  // Trigger the base class as well...
  PrivilegedManagedProcess::start();
  // Add a change filter to the setup...
  try {
    addChangeFilter( createLocalTypeFilter<Action>(WorkingMemoryOperation(ADD)), 
		     this );


    assert(m_scReceiver == NULL);
    m_scReceiver 
      = new MemberFunctionChangeReceiver<PretendManipulator>(this, 
							     &PretendManipulator::sceneChangedAdded);
    addChangeFilter(createGlobalTypeFilter<SceneChanged>(cdl::ADD),
		    m_scReceiver);
    

  }
  catch( const SubarchitectureProcessException & ex ) {
    cout << "\n Starting trouble in PretendManipulator...\n";
    std::abort( );
  }  

}

void 
PretendManipulator::sceneChangedAdded(const cast::cdl::WorkingMemoryChange& _wmc) {
  m_scAddress = _wmc.m_address;
  removeChangeFilter(m_scReceiver, cdl::DELETE_RECEIVER);
  m_scReceiver = NULL;
}


/**
 * Function that responds to working memory change...
 */
void 
PretendManipulator::workingMemoryChanged( const WorkingMemoryChange& _wmc ) {
  try {
    // sleepProcess(1000);
    
    // Get the action to perform...
    shared_ptr< const CASTData<Action> > action_ptr = 
      getWorkingMemoryEntry<Action>( _wmc.m_address );
    Action action = *( action_ptr->getData() );

    srand ( time(NULL) );

    double r((double)rand() / (double)RAND_MAX);
    
    if (r >= m_failureChance) {
      log( "Performing action now -- start praying..." );
      performAction(action);
    }
    else {
      log("Not performing action");
    }

    
    // Manually fix the status bits of the action variable...
    action.m_status = PlanningStatus( COMPLETE );
    action.m_succeeded = TriBool( triTrue );
    
    // Overwrite the working memory entries...
    overwriteWorkingMemory( string(_wmc.m_address.m_id), 
			    new Action( action ) , 
			    BLOCKING);

    log("overwritten action");
  }
  catch( const SubarchitectureProcessException & ex ) {
    cout <<"\n Change detection in PretendManipulator failed...\n";
    cout << ex.what() <<endl;
    std::abort();
  }
}



/**
 * Function that simulates the action performance...
 */
void PretendManipulator::performAction( const Action & _action ) {
  shared_ptr< const PickAndPlaceCmd > pnp = 
    getWorkingMemoryEntry<PickAndPlaceCmd>(_action.m_action.m_address)->getData();
  
  //copy object to edit
  SceneObject * obj = 
    new SceneObject(*(getWorkingMemoryEntry<SceneObject>(pnp->m_objectPointer.m_address)->getData()));
 

  //sleep for a few seconds to make it more "real"
  for(unsigned int i = 1; i <= 3; ++i) {
    log("%d...",i);
    sleepProcess(1000);  
  }

  log("PretendManipulator::performAction original centroid: %f %f %f", obj->m_bbox.m_centroid.m_x, obj->m_bbox.m_centroid.m_y, obj->m_bbox.m_centroid.m_z);
  obj->m_pose = pnp->m_targetPose;
  obj->m_bbox.m_centroid = pnp->m_targetPose.m_position;
  log("PretendManipulator::performAction new centroid: %f %f %f", obj->m_bbox.m_centroid.m_x, obj->m_bbox.m_centroid.m_y, obj->m_bbox.m_centroid.m_z);
  overwriteWorkingMemory(pnp->m_objectPointer.m_address,  obj, cdl::BLOCKING);
  obj = NULL;

  SceneObjectUpdate * update = new SceneObjectUpdate();
  update->m_updates.m_position = true;
  update->m_updates.m_colour = false;
  update->m_updates.m_shape = false;
  update->m_updates.m_size = false;
  update->m_soID = CORBA::string_dup(pnp->m_objectPointer.m_address.m_id);
  addToWorkingMemory(newDataID(), string(pnp->m_objectPointer.m_address.m_subarchitecture), update, cdl::BLOCKING);
  update = NULL;

  
  //copy object to edit
  assert(m_scReceiver == NULL);

  SceneChanged * sc = 
    new SceneChanged(*(getWorkingMemoryEntry<SceneChanged>(m_scAddress)->getData()));
  sc->m_sceneChanging = false;
  sc->m_sceneChanged = true;
  sc->m_sceneProcessed = true;
  overwriteWorkingMemory(m_scAddress, sc);
  sc = NULL;
  



}

/**
 * Dummy function not currently in use...
 */
void PretendManipulator::taskAdopted( const std::string& _taskID ) {

}



/**
 * Dummy function not currently in use...
 */
void PretendManipulator::taskRejected( const std::string& _taskID ) {

}
  


/**
 * Default runComponent function for the CASTComponent that does
 * nothing here...
 */
void PretendManipulator::runComponent() {
 
}


void PretendManipulator::configure(std::map<std::string,std::string> & _config) {
  PrivilegedManagedProcess::configure(_config);

  std::map<std::string,std::string>::const_iterator i =  _config.find("--failure");
  if(i != _config.end()) {
    m_failureChance = atof(i->second.c_str());
  }
  else {
    m_failureChance = 0.5;
  }

}

/**
 * The function called to create a new instance of our component...
 */
extern "C" {
  FrameworkProcess* newComponent( const std::string& _id ) {
    return new PretendManipulator(_id);
  }
}

