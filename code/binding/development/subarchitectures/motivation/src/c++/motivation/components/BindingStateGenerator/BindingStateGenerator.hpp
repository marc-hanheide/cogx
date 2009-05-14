#ifndef MOTIVATION_MONITOR_H_
#define MOTIVATION_MONITOR_H_

#include "BasicTranslators.hpp"

#include <binding/idl/BindingData.hh>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <binding/utils/GraphLoader.hpp>
#include <motivation/idl/MotivationData.hh>
#include <planning/util/TemporaryPlanningState.hpp>

#include <ext/hash_set>


using namespace cast;
using namespace cast::cdl;
using namespace std;
using namespace boost;
using namespace Binding;
  




/**
 * A component that monitors the proxies of other subarchitectures
 * (specified by the --monitor a,b,c flag) to keep track of the
 * proxies that should be used to generate goals for the system. If
 * a consisten planning variable is required for a particular
 * subarchitecture, specify the subarch with the --variables flag.
 *
 * TODO keep a basic state the is then extended.
 * 
 */
class BindingStateGenerator : 
  public cast::ManagedProcess,
  public Binding::AbstractBindingWMRepresenter
{
private:
  typedef StringMap< std::string >::map StringStringMap;

  typedef __gnu_cxx::hash_set<string> StringSet;

  typedef StringMap< StringSet >::map StringSetMap;

public:

  typedef  boost::function<bool (const UnionPtr & _union, const string & _var, TemporaryPlanningState & _state)> BUFWrapper;

  BindingStateGenerator(const string &_id);
    
  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(map<string,string> & _config);
  
  const string & variableName(const Binding::UnionPtr & _union) const {
    
    for(std::set<std::string>::const_iterator pid = _union->proxyIDs().begin();
	pid != _union->proxyIDs().end(); ++pid) {
      StringStringMap::const_iterator vn = m_proxy2var.find(*pid);
      if(vn != m_proxy2var.end()) {
	return vn->second;
      }
    }

    return _union->id();
  }

  const string & variableName(const std::string & _id) const {
      
    StringStringMap::const_iterator vn = m_proxy2var.find(_id);
    if(vn != m_proxy2var.end()) {
      return vn->second;
    }
 
    return _id;
  }

protected:
  void taskAdopted(const string &_taskID){}
  void taskRejected(const string &_taskID){}
  void runComponent();

private:

  void 
  avmListAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  void 
  generationCompetenceAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  /**
   * Called when planning state is required.
   */
  void 
  stateRequested(const cdl::WorkingMemoryChange& _wmc);

  /**
   * Called when planning state is required.
   */
  void 
  dummyTrigger(const cdl::WorkingMemoryChange& _wmc);


  /**
   * Perform state generation and store results.
   */
  void generateAndStoreState();


  /**
   * Create the minimum set of variables covering all of the
   * monitored proxies.
   *
   * For each union, ensure that all of its proxies are mapped to
   * the same gensym.
   */
  void updateVariables(const UnionSet & _unions);


  /**
   * Process the unions of all monitored proxies to generate a state
   * description.
   */
  void generateState(const UnionSet & _unions, 
		     TemporaryPlanningState & _state);

  void registerFunctors();

  /**
   * Adds prestored competences to the planning state. 
   */
  void addCompetences(const std::string & _agent, 
		      TemporaryPlanningState & _state) const;

  /**
   * Create and store a new BasicBindingStateGenerator using the two
   * templated classes for functionality.
   */
  template <class Applicable, class Translator>
  void registerBasicUnionGenerator() {
    m_basicFunctors.insert(new BasicBindingStateGenerator<Applicable, Translator>());
  }

  
  /**
   * Create and store a new BasicBinaryRelationStateGenerator using
   * the two templated classes for functionality.
   */
  template <class Applicable, class Translator>
  void registerBinaryRelationUnionGenerator() {
    m_simpleRelFunctors.insert(new BasicBinaryRelationStateGenerator<Applicable, Translator>());
  }


  /**
   * Create and store a new BasicBinaryRelationStateGenerator using
   * the a relation label check for applicability, and the template
   * class for translation.
   */
  template <class Translator>
  void registerBinaryRelationUnionGenerator(const string & _label) {
    m_simpleRelFunctors.insert(new BasicBinaryRelationStateGenerator<RelationLabelApplicable, Translator>(_label));
  }

  ///utility object for loading data
  BindingGraphHandler m_handler;


  ///Translators for converting from single unions
  set<AbstractBasicStateGenerator *> m_basicFunctors;

  ///Translators for converting from binary relation unions
  set<AbstractBinaryRelationStateGenerator *> m_simpleRelFunctors;

  ///Cached planning state, generated whenever binding is stable
  TemporaryPlanningState m_cachedState;

  ///the robot's self ;)
  ObjectDeclaration m_self;

  ///Receiver to get teh address of the avm list
  cast::WorkingMemoryChangeReceiver * m_avmReceiver;

  ///location of avm on wm
  std::string m_avmID;

  StringStringMap m_proxy2var;

  ///store which subarch can generate which competence
  StringSetMap m_subarch2competences;


}; // monitor

#endif //MOTIVATION_MONITOR_H_

