/**
 * Temporary Planning State...
 *
 * This state is used to make it easier to deal with the complicated
 * CORBA based planning state by converting it to a STL vector
 * representation...
 *
 * @author Mohan
 */

#ifndef TEMPORARY_PLANNING_STATE_H_
#define TEMPORARY_PLANNING_STATE_H_



#include <planning/idl/PlanningData.hh>
#include <planning/idl/Planner.hh>

#include <planning/util/PlanningUtils.hpp>

#include <iostream>
#include <set>


using namespace std; 
using namespace Planner; 
using namespace planning::autogen;


/**
 * A class to handle the conversions from the TemporaryPlanningState
 * back to the PlanningState that can be used in the Planning
 * module...
 */

class TemporaryPlanningState {

public:
  typedef set<Fact,FactComparator> FactSet;
  typedef set<ObjectDeclaration, ObjectDeclarationComparator> ObjectDeclarationSet;


  TemporaryPlanningState();
  ~TemporaryPlanningState();

  /**
   * Converts the current state (in the form of the temporary planning
   * structure) into the state used outside the planning domain, the
   * 'PlanningState'...
   */
  PlanningStateLists toPlanningState();

  /**
   * Function gets the facts and objects currently available as
   * 'TemporaryPlanningState' and puts it into the input
   * 'PlanningState'...
   */
  void toPlanningState( PlanningStateLists& _state );

  /**
   * Function extends the current state (TemporaryPlanningState) with
   * the contents of the PlanningState provided as input...
   */
  void extend( PlanningStateLists _state );

  /**
   * Empties the state.
   */
  void clear() {
    m_factList.clear();
    m_objectList.clear();
  }

  FactSet m_factList;
  ObjectDeclarationSet m_objectList;

protected:
 

private:
  
  PlanningStateLists m_stateLists;
};



#endif
