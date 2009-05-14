/**
 * Simple Trigger to get everything else started...
 *
 * @author Mohan
 * @date October 19 2007
 */

#ifndef TRIGGER_H_
#define TRIGGER_H_

#include <vector>
#include <map> 
#include <algorithm>

#include <planning/idl/Planner.hh>
#include <planning/idl/PlanningData.hh>
#include <planning/util/PlanningDataTranslator.hpp>

#include <cast/architecture/WorkingMemoryReaderProcess.hpp>
#include <cast/architecture/WorkingMemoryChangeReceiver.hpp>

#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include <cast/architecture/SubarchitectureProcessException.hpp>


// Unfortunately the namespaces have to be set in this crude manner...
using namespace std; 
using namespace Planner; 
using namespace planning::autogen; 
using namespace cast;
using namespace cast::cdl;


// Another hack to set the planning domain for now...
//static string VISUAL_PLANNING_DOMAIN = "./subarchitectures/planning.sa/config/domains/visual-planning-domain.mapl";


static string PLAYMATE_AGENT_NAME = "MrChips";
static string PLAYMATE_DOMAIN_NAME = "playmate";
static string PLAYMATE_DOMAIN_FILE = "subarchitectures/planning.sa/src/python/mapsim/domains/playmate/mapl_files/domain.mapl";


/**
 * Component that forms part of the example code under
 * Planning. Writes an initial state to working memory, then triggers
 * a planning process on the state...
 * 
 * @author Mohan, Nick
 */

class Trigger : 
  public PrivilegedManagedProcess,
  public WorkingMemoryChangeReceiver {

public:
  /**
   * The constructor and destructor...
   */  
  Trigger( const std::string _id );
  ~Trigger();

  /**
   * Just a dummy start function to register a change filter...
   */
  void start();
  
  void workingMemoryChanged(const WorkingMemoryChange & _wmc);

 protected:
  /**
   * Function that runs the component, i.e. builds the state, writes
   * it into the working memory and requests a planning process...
   */
  void runComponent();

 private:

  /**
   * Self-explanatory -- build the notion of state from the facts and
   * objects of the domain...
   */
  void buildState();

  /**
   * Function to add the built state to the working memory...
   */
  void writeState();

  /**
   * Just a precautionary funtion to ensure that all objects are
   * written out to the working memory...
   */
  void flush();

  /**
   * Function that requests a planning process for a given goal,
   * subarchitecture and domain...
   */
  void requestPlanningProcess( const std::string& _goal );
  
  /**
   * Function that is not exactly required here...
   */
  void taskAdopted( const std::string& _taskID );

  /**
   * Function that is not exactly required here...
   */
  void taskRejected( const std::string& _taskID );


  /**
   * Adds an object declaration to the initial state.
   */
  void addObjectDeclaration(const std::string& _name,
			    const std::string& _type);
  
  /**
   * Adds an object declaration to the initial state.
   */
  void addFact(const std::string& _name,
	       const std::string _args[],
	       const unsigned int & _arglen,
	       const std::string& _value = "true");

  // The containers for the facts and the objects...
  vector<Fact> m_facts;
  vector<ObjectDeclaration> m_objects;

};







#endif
