#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "BindingGroupManager.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"
#include "cast/core/CASTUtils.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include <binding/utils/GraphLoader.hpp>
#include <binding/utils/Predicates.hpp>

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>
#include <boost/next_prior.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach

using namespace std;
using namespace boost;
using namespace boost::assign;
using namespace cast;
using namespace BindingFeatures;
/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::BindingGroupManager(_id);
  }
}

namespace Binding {

BindingGroupManager::BindingGroupManager(const string &_id) : 
  
  WorkingMemoryAttachedComponent(_id),
  //AbstractBinder(_id),
  AbstractMonitor(_id)//,
//  AbstractBindingWMRepresenter(dynamic_cast<WorkingMemoryReaderProcess&>(*this))
{ 
}

void
BindingGroupManager::start() {
  
  AbstractMonitor::start();
  
  addChangeFilter(createLocalTypeFilter<BindingFeatures::details::GroupDetails>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::groupDetailsAdded));
  
  /*  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::ADD),
      new MemberFunctionChangeReceiver<BindingGroupManager>(this,
      &BindingGroupManager::proxyAdded));
  */
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::proxyUpdated));

  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::proxyDeleted));
  
  m_sourceID = m_subarchitectureID; // we assume that this component is rrunning in the binding subarch
}

BindingGroupManager::~BindingGroupManager() {}

void
BindingGroupManager::groupDetailsAdded(const cast::cdl::WorkingMemoryChange& _wmc) 
{
  using cast::operator<<;
  const BindingFeatures::details::GroupDetails details(*loadBindingDataFromWM<BindingFeatures::details::GroupDetails>(_wmc));
  m_groupProxyIDs.insert(string(details.m_groupProxyID));
}
  
/*void 
BindingGroupManager::proxyAdded(const cdl::WorkingMemoryChange& _wmc) 
{
  string id(_wmc.m_address.m_id);
  if(m_proxyLocalCache[id].type() == BindingData::GROUP)
    m_groupProxyIDs.insert(id);
}*/

void 
BindingGroupManager::proxyUpdated(const cdl::WorkingMemoryChange& _wmc) 
{
  string id(_wmc.m_address.m_id);
  set<string>::const_iterator group_id = m_groupProxyIDs.find(id);
  //  std::set<std::string> 
  //  cast::StringMap<std::set<std::string> >::map::
  set<string>::const_iterator individual_id = m_individualProxyIDs.find(id);
  if(group_id == m_groupProxyIDs.end() &&
     individual_id == m_individualProxyIDs.end()) // not a group or individual, so never mind
    return;
  ProxySet group; // should point to the group proxy (i.e. size of set == 1)
  ProxySet individuals; // should point to all the individuals
  BindingGraphHandler handler(*this);  
  do { //while(!true_for_all(handler.allLoadedProxies(), ConsistencyCheck<ProxyPtr>(*this)));
    handler.reloadAllLoadedData(); // in case sth was inconsistent, first iteration the loaded data is empty
    if(group_id != m_groupProxyIDs.end()) {
      group = handler.loadProxies(*group_id);
      assert(group.size() == 1);
    } else {
      ProxySet one_individual = handler.loadProxies(*individual_id);
      group = handler.extractProxiesFromProxies(one_individual, GroupProxyFromSingularExtractor<ProxyPtr>());
      assert(group.size() == 1);
    }
    individuals = handler.extractProxiesFromProxies(group,SingularProxiesFromGroupExtractor<ProxyPtr>());
    const LBindingProxy& group_proxy(*(group.begin()->second));
    assert(true_for_all(individuals,hasFeature<ProxyPtr,BindingFeatures::Singular>()));
    assert(true_for_all(group, TypeCheck<ProxyPtr>(BindingData::GROUP)));
    assert(group.size() == 1);
    log("group " + lexical_cast<string>(group) + " and individuals loaded " + lexical_cast<string>(individuals));
    if(true_for_all(group, ProxyStateChecker(BindingData::BOUND)) &&
       true_for_all(individuals, ProxyStateChecker(BindingData::BOUND))
       ) { // only spawn off when all are bound      
      // now, get the individuals that are bound only to themselves or
      // other singulars (other singulars are not counted as a binding
      // in this case as it would result in infinitely many singulars if
      // there are two matching groups)
      ProxySet free_individuals = individuals | // we already know that the proxies are singulars, so if they are bound to nothing else, they're free
	(//ProxyBindingsCountCheck<>(1) || 
	 FreeSingular(handler)); 
      log("free individuals: " + lexical_cast<string>(free_individuals));
      if(free_individuals.empty()) { // then we need a new one
	m_individualProxyIDs.insert(_create_and_store_singular(group_proxy));
      } 
      else if(free_individuals.size() >= 2) { // too many individuals remove all except one
	set<string> individual_ids;
	foreach(ProxySet::value_type id, individuals) {
	  individual_ids.insert(id.first);
	}
	for(ProxySet::const_iterator ind = boost::next(free_individuals.begin()); // skip one, delete the rest
	    ind != free_individuals.end(); 
	    ++ind) {
	  m_individualProxyIDs.erase(ind->first);
	  individual_ids.erase(ind->first);
	  deleteExistingProxy(ind->first);
	}
	string detailID = group_proxy.getGroupDetailsID();
	const BindingFeatures::details::GroupDetails& details(group_proxy.getGroupDetails());
	BindingFeatures::details::GroupDetails* new_details = new BindingFeatures::details::GroupDetails(details);
	new_details->m_groupMemberProxyIDs.length(individual_ids.size());
	unsigned int i = 0;
	foreach(string id, individual_ids) {
	  new_details->m_groupMemberProxyIDs[i++] = CORBA::string_dup(id.c_str());
	}
	overwriteWorkingMemory(detailID,
			       getBindingSA(),
			       new_details,
			       cast::cdl::BLOCKING);
      }
    } // end of if all BOUND
  } while(!true_for_all(handler.allLoadedProxies(), ConsistencyCheck<ProxyPtr>(*this)));
  bindNewProxies();
}
  
void 
BindingGroupManager::proxyDeleted(const cdl::WorkingMemoryChange& _wmc) 
{
  const string id(_wmc.m_address.m_id);
  m_groupProxyIDs.erase(id);
  m_individualProxyIDs.erase(id);
}

BindingData::FeaturePointers 
BindingGroupManager::_copy_feature_pointers(const LBindingProxy& _proxy, 
					    const set<string>& _exclude) 
{
  BindingData::FeaturePointers ptrs;
  for(unsigned int i = 0 ; 
      i < _proxy->m_proxyFeatures.length(); 
      ++i) {
    if(_exclude.find(string(_proxy->m_proxyFeatures[i].m_type)) == _exclude.end()) {
      ptrs.length(ptrs.length() + 1);
      ptrs[ptrs.length() - 1] = _proxy->m_proxyFeatures[i];
    }
  }
  return ptrs;
}

string
BindingGroupManager::_create_and_store_singular(const LBindingProxy& _group)
{
  
  log("spawning off individual for group proxy: " + _group.id());
  assert(_group->m_type == BindingData::GROUP);

  startNewBasicProxy();
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  static set<string> exclude;
  if(exclude.empty()) {
    exclude.insert(ontology.featureName(typeid(Group)));
    //exclude.insert(ontology.featureName(typeid(CreationTime)));
    exclude.insert(ontology.featureName(typeid(DebugString)));
    exclude.insert(ontology.featureName(typeid(ThisProxyID)));
  }
  m_currentlyBuiltProxy->m_proxyFeatures = _copy_feature_pointers(_group, exclude);
  BindingFeatures::Singular sing_feat;
  sing_feat.m_groupID = CORBA::string_dup(_group.id().c_str()); 
  addFeatureToCurrentProxy(sing_feat);
  // now, store the proxy
  string singularID = storeCurrentProxy(false); // don't store system features
  
  string detailID = _group.getGroupDetailsID();
  const BindingFeatures::details::GroupDetails& details(_group.getGroupDetails());
  BindingFeatures::details::GroupDetails* new_details = new BindingFeatures::details::GroupDetails(details);
  new_details->m_groupMemberProxyIDs.length(details.m_groupMemberProxyIDs.length() + 1);
  for(unsigned int i = 0 ; i < details.m_groupMemberProxyIDs.length() ; ++i) {
    new_details->m_groupMemberProxyIDs[i] = CORBA::string_dup(string(details.m_groupMemberProxyIDs[i]).c_str());
  }
  new_details->m_groupMemberProxyIDs[details.m_groupMemberProxyIDs.length()] = CORBA::string_dup(singularID.c_str());
  overwriteWorkingMemory(detailID,
			 getBindingSA(),
			 new_details,
			 cast::cdl::BLOCKING);
  /*  for(unsigned int i = 0; i < _group.inPorts().m_ports.length() ; ++i) {
      _copy_relation_proxy(string(_group.inPorts().m_ports[i].m_proxyID),singularID);
      }
  */
  
  log("and the new individual proxy is: " + singularID);
  return singularID;
}

/*
string
BindingGroupManager::_copy_relation_proxy(const string& _relationID, 
					  const string& _singularID)
{
  
  log("spawning off relation to group proxy: " + _relationID);
  
  const LBindingProxy& relation(m_proxyLocalCache[_relationID]);
  log("1: " + _relationID);
  assert(relation->m_type == BindingData::RELATION);

  startNewRelationProxy();
  
  set<string> exclude;
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  exclude.insert(ontology.featureName(typeid(CreationTime)));
  exclude.insert(ontology.featureName(typeid(DebugString)));
  exclude.insert(ontology.featureName(typeid(ThisProxyID)));
  
  m_currentlyBuiltProxy->m_proxyFeatures = _copy_feature_pointers(relation, exclude);

  // copy outports
  m_currentlyBuiltProxy->m_outPorts.m_ports.length(relation->m_outPorts.m_ports.length());
  assert(m_single2group.find(_singularID) != m_single2group.end());
  string groupID = m_single2group.find(_singularID)->second;
  
  for(unsigned int i = 0 ; i < m_currentlyBuiltProxy->m_outPorts.m_ports.length() ; ++i) {
    string toProxy(relation->m_outPorts.m_ports[i].m_proxyID);
    if(toProxy == groupID) {
      toProxy = _singularID;
    }
    m_currentlyBuiltProxy->m_outPorts.m_ports[i] = relation->m_outPorts.m_ports[i];
    m_currentlyBuiltProxy->m_outPorts.m_ports[i].m_proxyID = CORBA::string_dup(toProxy.c_str());
    m_currentlyBuiltProxy->m_outPorts.m_ports[i].m_ownerProxyID = CORBA::string_dup(m_currentProxyID.c_str());
  }
  updateInports(m_currentlyBuiltProxy->m_outPorts);
  addCreationTimeToCurrentProxy(); 
  
  // now, store the proxy
  string newRelationID = storeCurrentProxy(false);
  
  log("and the new individual proxy is: " + newRelationID);
  return newRelationID;
}
*/

  
void
BindingGroupManager::configure(map<string,string>& _config)
{
  AbstractMonitor::configure(_config);
}
  
} // namespace Binding 


