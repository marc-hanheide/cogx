/**
 * Planning Data Translator...
 *
 * A class that translates between a list of working memory entries
 * and an arbitrary number of object declarations and facts in a
 * planning state -- exact copy of the same file in Java...
 *
 * @author Mohan
 */

#ifndef PLANNING_DATA_TRANSLATOR_H_
#define PLANNING_DATA_TRANSLATOR_H_

#include <iostream>

#include <cast/core/CASTData.hpp>
#include <planning/components/abstr/PlanningStateGenerator.hpp>
#include "TemporaryPlanningState.hpp"

using namespace std; 
using namespace Planner; 
using namespace planning::autogen;
using namespace cast::cdl;


/**
 * First the base class from which the class below will be derived --
 * essentially required because we need to have separate classes for a
 * variety of types but need them all to have a similar template...
 */
class PlanningDataTranslatorBase {
public:
  PlanningDataTranslatorBase() {};
  virtual ~PlanningDataTranslatorBase() {};

  virtual TemporaryPlanningState toPlanningState( const std::string& subarchitecture, 
						  const std::string & type, 
						  PlanningStateGenerator& psg ) = 0;
};



/**
 * A class that translates between a list of working memory entries,
 * and a number of object declarations and facts, into a planning
 * state...
 * 
 * @author Mohan
 */
template <class T> 
class PlanningDataTranslator : public PlanningDataTranslatorBase {
  
public:
  PlanningDataTranslator() {};

  virtual ~PlanningDataTranslator() {};
  
  /**
   * The methods receives a list of entries from working memory and
   * must return them as a planning state.
   * 
   */
  virtual TemporaryPlanningState toPlanningState( vector< shared_ptr< const CASTData<T> > > _wme ) = 0;

  /**
   * An overloaded version of the function above, to be used in the
   * PlanningStateGenerator -- takes the particular subarchitecture
   * key and the data translator key as inputs...
   */
  virtual TemporaryPlanningState toPlanningState( const std::string& subarchitecture, 
						  const std::string & type, 
						  PlanningStateGenerator& psg ) {
    // std::cout << "In PDT:toPlanningState...\n";
    std::vector< shared_ptr< const CASTData<T> > > genWME;
    // std::cout << "Before getWorkingMemoryEntries...\n"; 
    psg.getWorkingMemoryEntries<T>( subarchitecture, 0, genWME );
    // std::cout << "In PDT:got WME...\n";
    return toPlanningState( genWME );
  };
  
};





#endif
