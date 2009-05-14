/**
 * Simple State Generator...
 *
 * @author Mohan
 * @date October 19 2007
 */

#ifndef SIMPLE_STATE_GENERATOR_H_
#define SIMPLE_STATE_GENERATOR_H_

#include <vector>
#include <map> 
#include <algorithm>

#include <planning/idl/Planner.hh>
#include <planning/util/PlanningDataTranslator.hpp>
#include <planning/util/TemporaryPlanningState.hpp>
#include <planning/components/abstr/PlanningStateGenerator.hpp>

/* #include <vision/idl/Vision.hh> */
/* #include <vision/VisionOntologyFactory.hpp> */

#include <cast/core/CASTData.hpp>

// Name Space settings -- pity it has to be done this way...
using namespace std; 
using namespace Planner; 
using namespace planning::autogen; 
using namespace cast::cdl;


/**
 * Generate a planning state from a working memory that contains
 * planning-related objects...
 * 
 * @author Mohan
 */
class SimpleStateGenerator : public PlanningStateGenerator {
 public:
  // First the contructor and the destructor...
  SimpleStateGenerator( const std::string _id );
  ~SimpleStateGenerator();
  
  /**
   * Just a function to get the process running...
   */
  void start();

  /**
   * Default runComponent function for the CASTComponent that does
   * nothing here...
   */
  void runComponent();

};



/**
 * A particular instance of the PlanningDataTranslator template that
 * is used to convert FACTs to the TemporaryPlanningState type,
 * i.e. to vector of strings...
 */
class FactTranslator : public PlanningDataTranslator<Planner::Fact> {
public:
  FactTranslator() {};
  ~FactTranslator() {};

   /**
   * Specific implementation of the function in the
   * PlanningDataTranslator<T> templated class -- converts working
   * memory entries into a planning state (FACTS)...
   */
  TemporaryPlanningState toPlanningState( vector< shared_ptr< const CASTData<Planner::Fact> > > _wme ) {

    // std::cout << "FT: translating facts...\n";
    TemporaryPlanningState state;
    // Copy over the contents of the working memory (the facts) into the
    // appropriate entries of the TemporaryPlanningState instance,
    // i.e. the vectors...
    if( _wme.empty() ) {
      return state;
    }
    for( size_t i = 0; i < _wme.size(); ++i ) {
      std::cout << "Fact i from: " << (_wme.at(i))->getID() << endl;
      state.m_factList.insert( *( (_wme.at(i))->getData() ) );
    }
    
    return state;
  };

};




/**
 * A particular instance of the PlanningDataTranslator template that
 * is used to convert OBJECTDECLARATIONs to the TemporaryPlanningState
 * type, i.e. to vector of OBJECT DECLARATIONs...
 */
class ObjectDeclarationTranslator : public PlanningDataTranslator<Planner::ObjectDeclaration> {
public:
  ObjectDeclarationTranslator() {};
  ~ObjectDeclarationTranslator() {};

  /**
   * Specific implementation of the function in the
   * PlanningDataTranslator<T> templated class -- converts working
   * memory entries into a planning state (OBJECTS)...
   */
  TemporaryPlanningState toPlanningState( vector< shared_ptr< const CASTData<Planner::ObjectDeclaration> > > _wme ) {
    // std::cout << "ODT: translating objects...\n";
    TemporaryPlanningState state;
    // Copy over the contents of the working memory (the objects) into
    // the appropriate entries of the TemporaryPlanningState instance,
    // i.e. the vectors...
    if( _wme.empty() ) {
      return state;
    }
    for( size_t i = 0; i < _wme.size(); ++i ) {
      // std::cout << "Object i: " << i << endl;
      state.m_objectList.insert( *( _wme.at(i)->getData() ) );
    }
    return state;
  };
  
};





#endif
