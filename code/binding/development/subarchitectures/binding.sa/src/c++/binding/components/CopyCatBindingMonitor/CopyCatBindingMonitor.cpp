#include "CopyCatBindingMonitor.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/utils/BindingUtils.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>
#include "cast/architecture/ChangeFilterFactory.hpp"

namespace Binding {

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
    return new CopyCatBindingMonitor::CopyCatBindingMonitor(_id);
  }
}

CopyCatBindingMonitor::CopyCatBindingMonitor(const string& _id) :
    WorkingMemoryAttachedComponent(_id),
    AbstractMonitor(_id),
    AbstractBindingWMRepresenter(dynamic_cast<WorkingMemoryReaderProcess&>(*this))
{
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  m_excludedFeatures += ontology.featureName(typeid(SourceID));
  m_excludedFeatures += ontology.featureName(typeid(ThisProxyID));
}

void 
CopyCatBindingMonitor::configure(std::map<std::string,std::string> & _config) 
{
  AbstractMonitor::configure(_config);
}

void 
CopyCatBindingMonitor::start() {
  AbstractMonitor::start();
  //addChangeFilter(BindingLocalOntology::BIND_THESE_PROXIES_TYPE,
  //cdl::ADD, 
  //cdl::LOCAL_SA,
  //new MemberFunctionChangeReceiver<CopyCatBindingMonitor>(this,
  //&CopyCatBindingMonitor::bindTheseProxiesAdded));
  addChangeFilter(createLocalTypeFilter<BindingData::BindTheseProxies>(cdl::ADD),
		  new MemberFunctionChangeReceiver<CopyCatBindingMonitor>(this,
									  &CopyCatBindingMonitor::bindTheseProxiesAdded));
  //addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE,
  //cdl::DELETE, 
  //cdl::LOCAL_SA,
  //new MemberFunctionChangeReceiver<CopyCatBindingMonitor>(this,
  //&CopyCatBindingMonitor::proxyDeleted));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<CopyCatBindingMonitor>(this,
									  &CopyCatBindingMonitor::proxyDeleted));
}


  
void 
CopyCatBindingMonitor::bindTheseProxiesAdded(const cdl::WorkingMemoryChange& _wmc)
{
  
  vector<BindingData::ProxyPorts> outports;
  if (m_sourceID == "") {
    m_sourceID = _wmc.m_address.m_subarchitecture;
  }
  shared_ptr<const BindingData::BindTheseProxies> 
    bindTheseProxies(loadBindingDataFromWM<BindingData::BindTheseProxies>(_wmc));
  for(unsigned int i = 0; i < bindTheseProxies->m_proxyIDs.length(); ++i) {
    string proxyID(bindTheseProxies->m_proxyIDs[i]);
    log("bindTheseProxy: " + lexical_cast<string>(i) + ": " + proxyID);
    if(!existsOnWorkingMemory(proxyID)) {
      // noop... someone deleted this one real fast...
    } else {
      const LBindingProxy& proxy(m_proxyLocalCache[proxyID]);
      const FeatureSetWithRepetitions& features(proxy.comparableFeatureSetWithRepetitions());
      {StringMap<string>::map::iterator itr(m_proxyIDMap.find(proxyID));
	if(itr != m_proxyIDMap.end()) {
	  // if the proxy already is replicated, we delete current
	  // replica to replace it woth updated copy (expensive, but
	  // safe for now)
	  //try{
	    //deleteExistingProxy(itr->second);
	 //   log("Already have this: " + itr->second);
	  //} catch(const DoesNotExistOnWMException& _e) {
	  //  log("tried to delete proxy: " + itr->second + " : " + _e.what());
	 // }
	  //itr->second = newDataID();
	} else {
	  itr = m_proxyIDMap.insert(make_pair(proxy.id(),newDataID())).first;
	} 
	if(m_createdProxyIDs.find(itr->second) == m_createdProxyIDs.end()) {
	  startNewProxy(proxy->m_type,itr->second);
	  m_createdProxyIDs.insert(itr->second);
	} else {
	  static BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
	  changeExistingProxy(itr->second, ontology.features()); // exclude ALL features...
	}
	//cout << "m_proxyIDMap" << " 1\n";
	//for_all(m_proxyIDMap,print_pair_fct<string,string>(cout,"->"));
      }
      for(FeatureSetWithRepetitions::const_iterator i = features.begin() ; i != features.end() ; ++i) {
	for(OneTypeOfFeaturesWithRepetitions::const_iterator j = i->second.begin() ; j != i->second.end() ; ++j) {
	  //log("added a " + (*j)->name() + "to copied feature");
	  if(m_excludedFeatures.find((*j)->name()) == m_excludedFeatures.end())
	    (*j)->addFeatureToCurrentProxy(*this, (*j)->truthValue());
	}
      }
      
      m_currentlyBuiltProxy->m_outPorts.m_ports.length(proxy->m_outPorts.m_ports.length());
      for(unsigned int i = 0 ; i < proxy->m_outPorts.m_ports.length() ; ++i) {
	StringMap<string>::map::iterator itr2(m_proxyIDMap.find(string(proxy->m_outPorts.m_ports[i].m_proxyID)));
	if(itr2 != m_proxyIDMap.end()) {
	  //itr2->second = newDataID();
	} else {
	  itr2 = (m_proxyIDMap.insert(make_pair(string(proxy->m_outPorts.m_ports[i].m_proxyID),newDataID()))).first;
	}
	m_currentlyBuiltProxy->m_outPorts.m_ports[i] = proxy->m_outPorts.m_ports[i];
	m_currentlyBuiltProxy->m_outPorts.m_ports[i].m_proxyID = CORBA::string_dup(itr2->second.c_str());
	//cout << "m_currentlyBuiltProxy->m_outPorts.m_ports["<<i<<"].m_proxyID: " << m_currentlyBuiltProxy->m_outPorts.m_ports[i].m_proxyID<<endl;
	m_currentlyBuiltProxy->m_outPorts.m_ports[i].m_ownerProxyID = CORBA::string_dup(currentProxyID().c_str());
	//cout << "m_currentlyBuiltProxy->m_outPorts.m_ports["<<i<<"].m_ownerProxyID:" << m_currentlyBuiltProxy->m_outPorts.m_ports[i].m_ownerProxyID <<endl;
      }
      outports.push_back(m_currentlyBuiltProxy->m_outPorts);

//      cout << "m_proxyIDMap " << " 2\n";
//      for_all(m_proxyIDMap,print_pair_fct<string,string>(cout,"->"));
	
      // store the original proxy as source data...
      SourceData sd;
      sd.m_type = CORBA::string_dup(typeName<BindingData::BindingProxy>().c_str());//BindingLocalOntology::BINDING_PROXY_TYPE.c_str());
      sd.m_address.m_id = CORBA::string_dup(proxyID.c_str());
      sd.m_address.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
      
      addFeatureToCurrentProxy(sd);
      storeCurrentProxy();
      
    }
  }
  for(vector<BindingData::ProxyPorts>::const_iterator i = outports.begin() ; i != outports.end() ; ++i)
    updateInports(*i);

  bindNewProxies();

}
  
void 
CopyCatBindingMonitor::proxyDeleted(const cdl::WorkingMemoryChange& _wmc)
{
  StringMap<string>::map::iterator itr(m_proxyIDMap.find(string(_wmc.m_address.m_id)));
  if(itr != m_proxyIDMap.end()) {
    // if the proxy already is replicated, we delete current
    // replica to replace it woth updated copy (expensive, but
    // safe for now)
    try{
      deleteExistingProxy(itr->second);
    } catch(const DoesNotExistOnWMException& _e) {
      log("in CopyCatBindingMonitor::proxyDeleted: tried to delete proxy: " + itr->second + " : " + _e.what());
    }
  }
}


} // namespace Binding
