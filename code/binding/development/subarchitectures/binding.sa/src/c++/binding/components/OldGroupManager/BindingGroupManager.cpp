#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/ontology/BindingOntologyFactory.hpp"
#include "binding/ontology/BindingLocalOntology.hpp"
#include "BindingGroupManager.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>


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
  AbstractMonitor(_id),
  AbstractBindingWMRepresenter(dynamic_cast<WorkingMemoryReaderProcess&>(*this))
{ 
  setOntology(&BindingOntologyFactory::getOntology());
}

void
BindingGroupManager::start() {
  
  AbstractMonitor::start();
  
  //addChangeFilter(BindingLocalOntology::BIND_THESE_PROXIES_TYPE, 
  //cdl::ADD, 
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingGroupManager>(this,
  //&BindingGroupManager::proxyAdded));
  addChangeFilter(createLocalTypeFilter<BindingData::BindTheseProxies>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::proxyAdded));
  
  //addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE, 
  //cdl::DELETE, 
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingGroupManager>(this,
  //&BindingGroupManager::proxyDeleted));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::proxyDeleted));

  //addChangeFilter(BindingLocalOntology::BINDING_UNION_TYPE, 
  //cdl::ADD, 
  //cdl::LOCAL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingGroupManager>(this,
  //&BindingGroupManager::unionAdded));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingUnion>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::unionAdded));

  //addChangeFilter(BindingLocalOntology::BINDING_UNION_TYPE, 
  //cdl::OVERWRITE, 
  //cdl::LOCAL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingGroupManager>(this,
  //&BindingGroupManager::unionUpdated));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingUnion>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::unionUpdated));

  //addChangeFilter(BindingLocalOntology::BINDING_UNION_TYPE, 
  //cdl::DELETE, 
  //cdl::LOCAL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingGroupManager>(this,
  //&BindingGroupManager::unionDeleted));
  
  addChangeFilter(createLocalTypeFilter<BindingData::BindingUnion>(cdl::DELETE), 
		  new MemberFunctionChangeReceiver<BindingGroupManager>(this,
									&BindingGroupManager::unionDeleted));
  
  m_sourceID = m_subarchitectureID; // we assume that this component is rrunning in the binding subarch
}

BindingGroupManager::~BindingGroupManager() {}
  
  
void 
BindingGroupManager::proxyAdded(const cdl::WorkingMemoryChange& _wmc) {
  // get the data from working memory
  try{
    shared_ptr<const BindingData::BindTheseProxies> 
      bindTheseProxies(loadBindingDataFromWM<BindingData::BindTheseProxies>(_wmc));
    
    for(unsigned int i = 0; i < bindTheseProxies->m_proxyIDs.length(); ++i) {
      string proxyID(bindTheseProxies->m_proxyIDs[i]);
      log(string("proxyID[") + lexical_cast<string>(i) + "] : " + proxyID);
    }
    
    for(unsigned int i = 0; i < bindTheseProxies->m_proxyIDs.length(); ++i) {
      string proxyID(bindTheseProxies->m_proxyIDs[i]);
      const LBindingProxy& proxy(m_proxyLocalCache[proxyID]);
      
      //assert(proxy->m_proxyState != BindingData::BOUND);
      
      if(proxy->m_type == BindingData::GROUP) {
	const string& groupID(proxyID);
	const LBindingProxy& group(proxy);
	
	log("proxy added, and it's a group: " + proxyID);
	
	if(m_group_info.find(groupID) == m_group_info.end()) {
	  _create_and_store_singular(groupID, group);
	} else {
	  log("but it has already been added");
	}
      }
    }
    bindNewProxies();
  }
  catch(const DoesNotExistOnWMException& _e) {
    log(string("DoesNotExistOnWMException caught in BindingGroupManager::proxyAdded: ") + _e.what());
  }
}
  
void 
BindingGroupManager::proxyDeleted(const cdl::WorkingMemoryChange& _wmc) {
  string proxyID(_wmc.m_address.m_id);
  map<string,GroupInfo>::iterator itr(m_group_info.find(proxyID));
  if(itr == m_group_info.end()) {
    return;
  }
  for(set<string>::const_iterator i = itr->second.m_group_member_ids.begin() ; 
      i != itr->second.m_group_member_ids.end(); ++i) {
    deleteExistingProxy(*i);
    m_single2group.erase(*i);
  }
  m_group_info.erase(itr);
}

void 
BindingGroupManager::unionAdded(const cdl::WorkingMemoryChange& _wmc) {
  string unionID(_wmc.m_address.m_id);
  log(unionID + " added");
  _check_union(unionID);
}
  
void 
BindingGroupManager::unionUpdated(const cdl::WorkingMemoryChange& _wmc) {
  string unionID(_wmc.m_address.m_id);
  log(unionID + " updated");
  _check_union(unionID);
}

void 
BindingGroupManager::unionDeleted(const cdl::WorkingMemoryChange& _wmc) {
}
  
void 
BindingGroupManager::_check_union(const string& _unionID) {
  try {
    const LBindingUnion& binding_union(m_unionLocalCache[_unionID]);
    if(binding_union->m_proxyIDs.length() == 1) {
      // do nothing, since this binding obviously does not bind
      // an unbound proxy to anything but itself
      log("binding_union->m_proxyIDs.length() == 1 for unionID" + _unionID);
    } else {
      set<string> new_non_singulars;
      for(unsigned int i = 0; i < binding_union->m_proxyIDs.length() ; ++i) {
	string proxyID(binding_union->m_proxyIDs[i]);
	log("testing if " + proxyID + " is a new singular for binding " + _unionID);
	if(m_uni2prox[_unionID].find(proxyID) == m_uni2prox[_unionID].end() && //i.e., it wasn't in the list
	   m_single2group.find(proxyID) == m_single2group.end()) { // i.e. it's a singular
	  log("It is!");
	  new_non_singulars.insert(proxyID);
	}
      }
      // now, only do sth if the binding's proxy list is
      // updated with a new proxy ID that isn't a singular!    
      if(!new_non_singulars.empty()) {
	//      set<string> groupIDs;
	for(unsigned int i = 0; i < binding_union->m_proxyIDs.length() ; ++i) {
	  string proxyID(binding_union->m_proxyIDs[i]);
	  set<string>::iterator unbound = m_unbound_singles.find(proxyID);
	  if(unbound != m_unbound_singles.end()) {
	    log(*unbound + " is unbound");
	    assert(m_single2group[*unbound] != "");
	    string groupID(m_single2group[*unbound]);
	    const LBindingProxy& group(m_proxyLocalCache[groupID]);

	    _create_and_store_singular(groupID, group);
	    
	    m_unbound_singles.erase(unbound); 
	  }
	}   
      }
      
      bindNewProxies();
    }
    
    // very stupid coding nw... getting tired...
    m_uni2prox.clear();
    bool any_singular = false;
    for(unsigned int i = 0; i < binding_union->m_proxyIDs.length() ; ++i) {
      m_uni2prox[_unionID].insert(string(binding_union->m_proxyIDs[i]));
      if(m_single2group.find(string(binding_union->m_proxyIDs[i])) == m_single2group.end())
	any_singular = true;
    }
    if(!any_singular)
      m_uni2prox.clear();
  }
  catch(const DoesNotExistOnWMException& _e) {
    log(string("DoesNotExistOnWMException caught in BindingGroupManager::_check_union: ") + _e.what());
  }
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
  
const BindingFeatures::Group&
BindingGroupManager::_retrieve_group_info(const LBindingProxy& _proxy)
{
  try{
    for(unsigned int i = 0 ; 
	i < _proxy->m_proxyFeatures.length(); 
	++i) {
      static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
      if(string(_proxy->m_proxyFeatures[i].m_type) == ontology.featureName(typeid(Group))) {
	const BindingFeatures::Group& group(extractIDLFeature<BindingFeatures::Group>(m_featureLoader.getFeature(_proxy->m_proxyFeatures[i])));
	return group;
      }
    }
    throw BindingException("ERROR in BindingGroupManager: a proxy " + _proxy.id() +" was of type GROUP, but didn't contain the required group feature.\n");
  }
  catch(const DoesNotExistOnWMException& _e) {
    throw(BindingException(string("DoesNotExistOnWMException caught in BindingGroupManager::_retrieve_group_info: ") + _e.what()));
  }
}
  

string
BindingGroupManager::_create_and_store_singular(const string& _groupID, 
						const LBindingProxy& _group)
{
  
  log("spawning off individual for group proxy: " + _groupID);
  
  assert(_group->m_type == BindingData::GROUP);

  startNewBasicProxy();
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  set<string> exclude;
  exclude.insert(ontology.featureName(typeid(Group)));
  //exclude.insert(ontology.featureName(typeid(CreationTime)));
  exclude.insert(ontology.featureName(typeid(DebugString)));
  exclude.insert(ontology.featureName(typeid(ThisProxyID)));
  
  m_currentlyBuiltProxy->m_proxyFeatures = _copy_feature_pointers(_group, exclude);
  
  BindingFeatures::Singular sing_feat;
  sing_feat.m_groupID = CORBA::string_dup(_groupID.c_str()); 
  map<string,GroupInfo>::iterator g = m_group_info.find(_groupID);
  if(g == m_group_info.end()) 
    g = m_group_info.insert(make_pair(_groupID,GroupInfo(_retrieve_group_info(_group).m_size))).first;
  sing_feat.m_elementNumber = g->second.m_current_size;
  addFeatureToCurrentProxy(sing_feat);
  //addCreationTimeToCurrentProxy(); // the creation time of the
  // individual is not the same as
  // the one for the group
  
  // now, store the proxy
  string singularID = storeCurrentProxy(false); // don't store system features
  // now some internal book-keeping
  g->second.m_current_size++;
  g->second.m_group_member_ids.insert(singularID);
  m_single2group[singularID] = _groupID;
  m_unbound_singles.insert(singularID);
  for(unsigned int i = 0; i < _group.inPorts().m_ports.length() ; ++i) {
    _copy_relation_proxy(string(_group.inPorts().m_ports[i].m_proxyID),singularID);
  }
  
  log("and the new individual proxy is: " + singularID);
  return singularID;
}

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


  
void
BindingGroupManager::configure(map<string,string>& _config)
{
  AbstractMonitor::configure(_config);
}
  
} // namespace Binding 


