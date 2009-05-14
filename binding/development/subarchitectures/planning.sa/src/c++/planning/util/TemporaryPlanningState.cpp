/**
 * Temporary Planning State...
 *
 * @author Mohan
 * @date October 19 2007
 */

#include "TemporaryPlanningState.hpp"


/**
 * Constructor, because something has to exist because it can be
 * used...
 */
TemporaryPlanningState::TemporaryPlanningState() {
}


/**
 * Now for the destructor...
 */
TemporaryPlanningState::~TemporaryPlanningState() {
  // cout << "TemporaryPlanningState::~TemporaryPlanningState() \n";

  // Remember to clear memory since we are no longer in Java...
  m_factList.clear();
  m_objectList.clear();
}



/**
 * Converts the current state (in the form of the temporary planning
 * structure) into the state used outside the planning domain, the
 * 'PlanningState'...
 */
PlanningStateLists TemporaryPlanningState::toPlanningState() {

  // Create the arrays into which the data has to be set...
  m_stateLists.m_facts.length( m_factList.size() );
  m_stateLists.m_objects.length( m_objectList.size() );

  {
    // Set the facts into the Planning State...
    int i = 0;
    for(FactSet::const_iterator fs = m_factList.begin(); 
	fs != m_factList.end(); ++fs) {    
      m_stateLists.m_facts[i++] = *fs;
    }
  }

  {
    int i = 0;
    for(ObjectDeclarationSet::const_iterator os = m_objectList.begin(); 
	os != m_objectList.end(); ++os) {    
      m_stateLists.m_objects[i++] = *os;
    }
  }
  // Return the Planning State out...
  return m_stateLists;
}



/**
 * Function gets the facts and objects currently available as
 * 'TemporaryPlanningState', combines it withe the values available in
 * the input 'PlanningState', and puts it back into the input
 * 'PlanningState'...
 */
void TemporaryPlanningState::toPlanningState( PlanningStateLists& _state ) {
  // First deal with the facts...
  FactSet tmpFacts;

  // Push the current known facts into the temp facts...
  tmpFacts.insert( m_factList.begin(), m_factList.end() );
  // Push the input state's facts into temp facts...
  for( size_t i = 0; i < (_state.m_facts).length(); ++i ) {
    tmpFacts.insert( _state.m_facts[i] );
  }

  {
    // Now push all the facts back into the input state...
    _state.m_facts.length( tmpFacts.size() );
    int i = 0;
    for(FactSet::const_iterator fs = tmpFacts.begin(); 
	fs != tmpFacts.end(); ++fs) {          
      _state.m_facts[i++] = *fs;
    }
  }


  // Remember to clear the vector -- no memory leaks please...
  tmpFacts.clear();

  // Then deal with the objects...
  ObjectDeclarationSet tmpObjects;

  // Push the current known object into the temp objects...
  tmpObjects.insert( m_objectList.begin(), m_objectList.end() );
  // Push the input state's objects into temp objects...
  for( size_t i = 0; i < (_state.m_objects).length(); ++i ) {
    tmpObjects.insert( _state.m_objects[i] );
  }

  {
  // Now push all the objects back into the input state...
    _state.m_objects.length( tmpObjects.size() );
    int i = 0;
    for(ObjectDeclarationSet::const_iterator os = tmpObjects.begin(); 
	os != tmpObjects.end(); ++os) {    
      _state.m_objects[i++] = *os;
    }
  }
  // Remember to clear the vector -- no memory leaks please...
  tmpObjects.clear();
}



/**
 * Function extends the current state (TemporaryPlanningState) with
 * the contents of the PlanningState provided as input...
 */
void TemporaryPlanningState::extend( PlanningStateLists _state ) {
  // Add to the facts known...
  for( size_t i = 0; i < _state.m_facts.length(); ++i ) {
    m_factList.insert( _state.m_facts[i] );
  }

  // Add to the objects known...
  for( size_t i = 0; i < _state.m_objects.length(); ++i ) {
    m_objectList.insert( _state.m_objects[i] );
  }

}
