#ifndef MOTIVATION_MONITOR_H_
#define MOTIVATION_MONITOR_H_

#include <binding/idl/BindingData.hh>
#include <binding/abstr/AbstractMonitor.hpp>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <binding/utils/GraphLoader.hpp>

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
 * 
 */
class MotivationMonitor : public AbstractMonitor {
private:
  typedef __gnu_cxx::hash_set<string> StringSet;
  typedef StringMap< std::pair<std::string, std::string> >::map StringPairMap;
  //  typedef StringMap<StringStringMap>::map SSStringMap;

public:

  MotivationMonitor(const string &_id);
    
  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(map<string,string> & _config);
  

protected:
  virtual void taskAdopted(const string &_taskID){}
  virtual void taskRejected(const string &_taskID){}
  virtual void runComponent();

private:
  /**
   * Called when the status struct is overwritten
   */
  void statusOverwritten(const cdl::WorkingMemoryChange& _wmc);

  /**
   * Called when any proxy is overwritten on binding wm.
   */
  void proxyOverwritten(const cdl::WorkingMemoryChange& _wmc);

  /**
   * Used internally to process proxies
   */
  void proxyOverwritten(const std::string& proxyID);

  /**
   * Called when any proxy is deleted on binding wm.
   */
  void deleteProxy(const std::string & _proxyID);

  /**
   * Called when any proxy is deleted on binding wm.
   */
  void proxyDeleted(const cdl::WorkingMemoryChange& _wmc);

  /**
   * Called when any proxy is deleted on binding wm.
   */
  void proxyScheduledForDeletion(const cdl::WorkingMemoryChange& _wmc);


  /**
   * Check whether this SourceID is to be monitored
   */
  bool monitoredSource(const BindingFeatures::SourceID& _sid) const;

  /**
   * Check whether the proxy at this address has been considered
   * previously for monitoring or exclusion
   */
  bool previouslyProcessed(const string& _proxyID) const;

  bool isMonitored(const string& _proxyID) const;
  bool isIgnored(const string& _proxyID) const;

  /**
   * Add this proxy to the set of proxies being monitored.
   */
  void monitorProxy(const ProxyPtr& _proxy, 
		    const BindingFeatures::SourceID& _sid);

  /**
   * Remove this proxy from the set of proxies being monitored.
   */
  void unmonitorProxy(const string& _proxyID);

  /**
   * Add this proxy to the set of proxies being ignored.
   */
  void ignoreProxy(const ProxyPtr& _proxy);

  /**
   * Remove this proxy from the set of proxies being ignored.
   */
  void unignoreProxy(const string& _proxyID);


  /**
   * Perform state generation and store results.
   */
  bool generateAndStoreMappings();


  /**
   * Create the minimum set of variables covering all of the
   * monitored proxies.
   *
   * For each union, ensure that all of its proxies are mapped to
   * the same gensym.
   */
  void updateVariables(const UnionSet & _unions);

  /**
   * Get variable name to use for this union. Based on the var name
   * for the proxies it contains.
   */
  const string & variableName(const UnionPtr & _union) const {
    //get the id of the first proxy
    assert(_union->proxyIDs().size() > 0);
    
    for(std::set<std::string>::const_iterator pids = _union->proxyIDs().begin();
	pids != _union->proxyIDs().end(); ++pids) {
      StringPairMap::const_iterator vn = m_variableNames.find(*pids);
      if(vn != m_variableNames.end() && vn->second.first != "") {
	return vn->second.first;
      }
    }

    //shouldn't ever really happen
    assert(true);
    return  _union->id();
  }

  /**
   * Gensym for variable names
   */
  string
  gensym() {
    std::ostringstream o;
    o<<getProcessIdentifier()<<m_variableCount++;
    return o.str();
  }
  
  /**
   * Store AddressVariableMappings on wm.
   */
  void 
  writeVariableMappings();

  void
  lockVariableMappings();

  void
  unlockVariableMappings();

  
  const BindingData::BinderStatus & binderStatus() {
    assert(m_statusID != "");
    return m_statusCache[m_statusID];
  }

  bool binderStable() {
    return binderStatus().m_stable && (binderStatus().m_bindingTasks == 0)
      && (binderStatus().m_boundProxies > 0);
  }  

  void generationCompetenceAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  /**
   * Called when something on binding wm changes. This means we should
   * check again whether binding is unstable, and lock up if it is a
   * new state change.
   */
  void stateChange();

  /**
   * Lock up variables.
   */
  void startVarUpdate();

  /**
   * Generate new mapping list, write to wm, then unlock variables.
   */
  void endVarUpdate();


  ///Number of variables created
  int m_variableCount;

  ///list of subarchitectures which will be monitored for motivation
  ///proxies
  StringSet m_monitoredSAs;    


  StringSet m_monitoredProxies;
  StringSet m_ignoredProxies;
    

  ///Maps from proxy ids to subarchitecture + variable names... is
  ///this always consistent?  for the pairs first = var name, second =
  ///subarchitecture
  StringPairMap m_variableNames;

  ///utility object for loading data
  BindingGraphHandler m_handler;

  ///wm id of variable list
  std::string m_variableListID;

  ///whether the variable list is locked or not
  bool m_variableListLocked;


  //status cache
  cast::CASTDataCache<BindingData::BinderStatus> m_statusCache;

  std::string m_statusID;

  ///something important has changed 
  
  
  
}; // monitor

#endif //MOTIVATION_MONITOR_H_

