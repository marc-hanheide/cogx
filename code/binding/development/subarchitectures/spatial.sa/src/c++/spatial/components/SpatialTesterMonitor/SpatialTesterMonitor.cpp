
#include "SpatialTesterMonitor.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/idl/BindingQueries.hh"
#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>
//#include <boost/assign/std/map.hpp>
//#include <boost/assign.hpp>
#include "cast/architecture/ChangeFilterFactory.hpp"

namespace Binding {
  using namespace boost;
  using namespace boost::assign;
  using namespace std;
  using namespace cast;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::SpatialTesterMonitor(_id);
  }
}


SpatialTesterMonitor::SpatialTesterMonitor(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  AbstractMonitor(_id),
  m_testFinished(true),
  m_handler(*this),
  m_retest(0),
  m_statusUpdates(0),
  m_statusStableUpdates(0)
{
}

static const unsigned int MANY_PROXIES = 24;
static const unsigned int FEW_PROXIES = 6;

void
SpatialTesterMonitor::start() {    
  AbstractMonitor::start();
  
  addChangeFilter(createGlobalTypeFilter<BindingData::BindingProxy>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialTesterMonitor>(this,
								  &SpatialTesterMonitor::bindingProxyAdded));
  addChangeFilter(createGlobalTypeFilter<BindingData::BindingProxy>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<SpatialTesterMonitor>(this,
								  &SpatialTesterMonitor::bindingProxyUpdated));

  addChangeFilter(createGlobalTypeFilter<BindingData::BindingUnion>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialTesterMonitor>(this,
								  &SpatialTesterMonitor::bindingUnionUpdated));

  addChangeFilter(createGlobalTypeFilter<BindingData::BindingUnion>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<SpatialTesterMonitor>(this,
								  &SpatialTesterMonitor::bindingUnionUpdated));

  
  addChangeFilter(createGlobalTypeFilter<BindingData::BinderStatus>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialTesterMonitor>(this,
								  &SpatialTesterMonitor::statusUpdated));

}

void 
SpatialTesterMonitor::configure(map<string,string> & _config) {
  AbstractMonitor::configure(_config);
  dynamic_cast<AbstractBindingWMRepresenter*>(this)->setBindingSubarchID(getBindingSA());
}

void SpatialTesterMonitor::runComponent() {
  m_sourceID = subarchitectureID();
  //things will take a while
  const_cast<SpatialTesterMonitor&>(*this).sleepProcess(180000);  
  testCompleteness();
}
  
void 
SpatialTesterMonitor::bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  //const LBindingProxy& binding_proxy(m_proxyLocalCache[id]);
  m_addSignalledProxyIDs.insert(id);
}

void 
SpatialTesterMonitor::bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  m_overwriteSignalledProxyIDs.insert(id);
  try {
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);      
  } catch(const DoesNotExistOnWMException& _e) {
    cerr << "Caught this in SpatialTesterMonitor::bindingProxyUpdated: " + string(_e.what());
    abort();
  }
}

void 
SpatialTesterMonitor::bindingUnionUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  try {
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);  
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in SpatialTesterMonitor::bindingUnionUpdated: " + string(_e.what()));
    abort();
  }

}

void 
SpatialTesterMonitor::statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  try {
    m_statusUpdates++;
    if(m_statusID.empty())
      m_statusID = _wmc.m_address.m_id;
    else
      assert(m_statusID == string(_wmc.m_address.m_id));
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);  
    if(m_status.m_stable)
      m_statusStableUpdates++;    
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in SpatialTesterMonitor::statusUpdated: " + string(_e.what()));
    abort();
  }

}


  
void 
SpatialTesterMonitor::testCompleteness() {

  try{

    //just load and count all proxues
    ProxySet allProxies(m_handler.allProxiesFromWM());
    //    m_retest = 11;

    awaitBinding(allProxies);

    if(allProxies.size() == 18) {
      successExit();
    }
    else {
      failExit();
    }

  }
  catch(const BindingException & _e) {
    log(string("Caught this while testing") + _e.what());
  }
  catch(const DoesNotExistOnWMException & _e) {
    log(string("Caught this while testing") + _e.what());
  }
  
  // reset the retesting if successExit was not called.
  m_retest = 0;  
  log("SpatialTesterMonitor::testCompleteness() just failed");
}
  
bool
SpatialTesterMonitor::binderEmptied() 
{
  return 
    m_status.m_stable && 
    m_status.m_unboundProxies == 0 &&
    m_status.m_boundProxies == 0;
}
  
bool
SpatialTesterMonitor::allBound() 
{  
  bool result = false;
  if ( 
       m_status.m_stable &&
       m_overwriteSignalledProxyIDs == m_addSignalledProxyIDs) {
    bool consistency_checked = false;
    while(!consistency_checked) {
      result = false;
      ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
      if(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND) && !ProxyUnionIDChecker(""))) { // return true after some assertions
	// some consistency checks:
	assert(true_for_all(proxies,
			    hasFeature<ProxyPtr,BindingFeatures::SourceID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::ThisProxyID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::CreationTime>() &&
			    (TypeCheck<ProxyPtr>(BindingData::GROUP) == hasFeature<ProxyPtr,BindingFeatures::Group>())
			    )
	       );
	assert(true_for_all(proxies,
			    featureCheck<ProxyPtr,BindingFeatures::ThisProxyID>(ThisProxyIDConsistencyChecker<>()))
	       );
	// make sure all ThisProxyID points correctly (well... not a dead certain test, but well)
	ProxySet proxies2 = m_handler.extractProxiesFromProxies(proxies,ProxyFromThisProxyIDExtractor<ProxyPtr>());
	assert(proxies == proxies2);
	// have a look at the singulars:
	proxies = proxies | hasFeature<ProxyPtr,BindingFeatures::Singular>();
	// and make sure they're all part of groups
	proxies = m_handler.extractProxiesFromProxies(proxies,GroupProxyFromSingularExtractor<ProxyPtr>());
	assert(true_for_all(proxies, TypeCheck<ProxyPtr>(BindingData::GROUP)));
	result = true;
      }
      if(true_for_all(proxies,ConsistencyCheck<ProxyPtr>(*this))) {
	consistency_checked = true;
      } else {
	m_handler.reloadAllLoadedData();
      }
    }
  }
  return result;
}
  
bool
SpatialTesterMonitor::allBound(const set<string>& _proxyIDs) 
{  
  bool result = false;
  if ( !_proxyIDs.empty() && 
       m_status.m_stable) {
    bool consistency_checked = false;
    while(!consistency_checked) {
      result = false;
      ProxySet proxies = m_handler.loadProxies(_proxyIDs);
      if(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND) && !ProxyUnionIDChecker(""))) { // return true after some assertions
	// some consistency checks:
	assert(true_for_all(proxies,
			    hasFeature<ProxyPtr,BindingFeatures::SourceID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::ThisProxyID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::CreationTime>() &&
			    (TypeCheck<ProxyPtr>(BindingData::GROUP) == hasFeature<ProxyPtr,BindingFeatures::Group>())
			    )
	       );
	assert(true_for_all(proxies,
			    featureCheck<ProxyPtr,BindingFeatures::ThisProxyID>(ThisProxyIDConsistencyChecker<>()))
	       );
	// make sure all ThisProxyID points correctly (well... not a dead certain test, but well)
	ProxySet proxies2 = m_handler.extractProxiesFromProxies(proxies,ProxyFromThisProxyIDExtractor<ProxyPtr>());
	assert(proxies == proxies2);
	// have a look at the singulars:
	proxies = proxies | hasFeature<ProxyPtr,BindingFeatures::Singular>();
	// and make sure they're all part of groups
	proxies = m_handler.extractProxiesFromProxies(proxies,GroupProxyFromSingularExtractor<ProxyPtr>());
	assert(true_for_all(proxies, TypeCheck<ProxyPtr>(BindingData::GROUP)));
	result = true;
      }
      if(true_for_all(proxies,ConsistencyCheck<ProxyPtr>(*this))) {
	consistency_checked = true;
      } else {
	m_handler.reloadAllLoadedData();
      }
    }
  }
  return result;
}


void
SpatialTesterMonitor::awaitBinding(const set<string>& _proxies) {
  awaitBinding(m_handler.loadProxies(_proxies));
}

void
SpatialTesterMonitor::awaitBinding(const ProxySet& _proxies) 
{
  ProxySet proxies = _proxies;
  while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
    log("all not bound");
    sleepProcess(1000);
    m_handler.reloadAllLoadedData();
    std::set<string> ids;
    insert_iterator<set<string> > inserter = std::inserter(ids,ids.begin());
    foreach(ProxySet::value_type prox , proxies) {
      inserter = prox.first;
    }
    proxies = m_handler.loadProxies(ids);
  }
}

} // namespace Binding
