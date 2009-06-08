#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "binding/feature-utils/Features.hpp"
#include "GarbageCollector.hpp"
#include <cast/core/CASTUtils.hpp>
#include "cast/architecture/ChangeFilterFactory.hpp"
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach


using namespace std;
using namespace boost;
using namespace boost::assign;
using namespace cast;
using namespace dot;
using namespace BindingData;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::GarbageCollector(_id);
  }
}

namespace Binding {

GarbageCollector::GarbageCollector(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id),
  m_confCalled(false)
{
  m_queueBehaviour = cdl::QUEUE;  
}

GarbageCollector::~GarbageCollector() {}

void 
GarbageCollector::start() {
  assert(m_confCalled);

  AbstractBinder::start();

  addChangeFilter(createLocalTypeFilter<DeletedBindingProxy>(cdl::ADD),
		  new MemberFunctionChangeReceiver<GarbageCollector>(this,
								     &GarbageCollector::deletedProxyAdded));
  addChangeFilter(createLocalTypeFilter<BindingUnion>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<GarbageCollector>(this,
								     &GarbageCollector::unionDeleted));
  addChangeFilter(createLocalTypeFilter<ProxyUnionScore>(cdl::ADD),
		  new MemberFunctionChangeReceiver<GarbageCollector>(this,
								     &GarbageCollector::bindingScoreAdded));
  addChangeFilter(createLocalTypeFilter<FeatureComparison>(cdl::ADD),
		  new MemberFunctionChangeReceiver<GarbageCollector>(this,
								     &GarbageCollector::featureComparisonAdded));
  addChangeFilter(createLocalTypeFilter<BinderStatus>(),
		  new MemberFunctionChangeReceiver<GarbageCollector>(this,
								     &GarbageCollector::statusUpdated));
  
  addChangeFilter(createLocalTypeFilter<ExplicitFeatureDeletionTask>(cdl::ADD),
		  new MemberFunctionChangeReceiver<GarbageCollector>(this,
								     &GarbageCollector::explicitFeatureDeletionAdded));

  if(m_policeMode != DISABLED) {
    const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    foreach(string feature, ontology.allFeatureNames()) {
      addChangeFilter(createChangeFilter(feature,
					 cdl::OVERWRITE,
					 "",
					 "",
					 "",
					 cdl::LOCAL_SA),
		      new MemberFunctionChangeReceiver<GarbageCollector>(this,
									 &GarbageCollector::reportIllegalSignal));
    }
    addChangeFilter(createLocalTypeFilter<DeletedBindingProxy>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<GarbageCollector>(this,
								       &GarbageCollector::reportIllegalSignal));
    
    addChangeFilter(createLocalTypeFilter<BinderStatus>(cdl::DELETE),
		    new MemberFunctionChangeReceiver<GarbageCollector>(this,
								       &GarbageCollector::reportIllegalSignal));
  }
}

void 
GarbageCollector::configure(map<string, string>& _config)
{
  m_policeMode = STRICT;
  AbstractBinder::configure(_config);
  map<string, string>::const_iterator itr = _config.find("-p");
  if(itr == _config.end())
    itr = _config.find("--police-mode");
  if(itr!=_config.end()) {    
    if(itr->second == "disabled")
      m_policeMode = DISABLED;
    else if (itr->second == "warning")
      m_policeMode = WARNING;
    else if (itr->second == "strict")
      m_policeMode = STRICT;
    else {
      cerr << "Illegal police mode: "  << itr->second << "(options are {strict,warning,disabled)" << endl;
      abort();
    }
  } 
  m_confCalled = true;
}


void GarbageCollector::runComponent() {
}

/*void 
GarbageCollector::deleteAllOnQueue() {
  while(!m_deletionQueue.empty()) {
    deleteOneOnQueue();
  }
}
*/
void  
GarbageCollector::deleteOneOnQueue() {
  assert(!m_deletionQueue.empty());
  log(string("deleting: ") + m_deletionQueue.front());
  assert(!m_deletionQueue.front().empty());
  deleteFromWorkingMemory(m_deletionQueue.front(),cdl::BLOCKING);
  m_deletionQueue.pop_front();  
}

void 
GarbageCollector::deletedProxyAdded(const cast::cdl::WorkingMemoryChange& _wmc) {
  shared_ptr<const DeletedBindingProxy> deleted = loadBindingDataFromWM<DeletedBindingProxy>(_wmc);
  m_deletionQueue.push_back(string(deleted->m_deletedProxy.m_bestUnionsForProxyID));
  assert(!m_deletionQueue.back().empty());
  m_deletionQueue.push_back(string(deleted->m_deletedProxy.m_nonMatchingUnionID));
  assert(!m_deletionQueue.back().empty());
  m_deletionQueue.push_back(string(deleted->m_deletedProxy.m_inPortsID));
  assert(!m_deletionQueue.back().empty());
  for(unsigned int i=0 ; i < deleted->m_deletedProxy.m_proxyFeatures.length() ; ++i ) {
    m_deletionQueue.push_back(string(deleted->m_deletedProxy.m_proxyFeatures[i].m_address));
    assert(!m_deletionQueue.back().empty());
  }
  // should be called only when binder is stable an not busy...
  
  StringMap<AssociatedIDs>::map::iterator itr = m_associatedToProxy.find(string(deleted->m_deletedProxyID));
  if(itr != m_associatedToProxy.end()) {    
    foreach(string id, itr->second.m_ids) {
      assert(!id.empty());
      m_deletionQueue.push_back(id);
    }
    m_associatedToProxy.erase(itr);
  }
}

/// deletes scores associated with the deleted union
void 
GarbageCollector::unionDeleted(const cast::cdl::WorkingMemoryChange& _wmc) 
{
  log("unionDeleted: " + string(_wmc.m_address.m_id));
  StringMap<AssociatedIDs>::map::iterator itr = m_associatedToUnion.find(string(_wmc.m_address.m_id));
  if(itr != m_associatedToUnion.end()) {    
    foreach(string id, itr->second.m_ids) {
      assert(!id.empty());
      m_deletionQueue.push_back(id);
    }
    m_associatedToUnion.erase(itr);
  }
}

void 
GarbageCollector::bindingScoreAdded(const cast::cdl::WorkingMemoryChange& _wmc)
{
  log("bindingScoreAdded: " + string(_wmc.m_address.m_id));
  shared_ptr<const ProxyUnionScore> score = loadBindingDataFromWM<ProxyUnionScore>(_wmc);
  m_associatedToUnion[string(score->m_unionID)].m_ids.insert(string(_wmc.m_address.m_id));
}

void 
GarbageCollector::featureComparisonAdded(const cast::cdl::WorkingMemoryChange& _wmc)
{
  shared_ptr<const FeatureComparison> score = loadBindingDataFromWM<FeatureComparison>(_wmc);
  m_associatedToProxy[string(score->m_proxyID)].m_ids.insert(string(_wmc.m_address.m_id));
}

void 
GarbageCollector::explicitFeatureDeletionAdded(const cast::cdl::WorkingMemoryChange& _wmc) 
{
  string task_id(_wmc.m_address.m_id);
  shared_ptr<const ExplicitFeatureDeletionTask> task_ptr(loadBindingDataFromWM<ExplicitFeatureDeletionTask>(task_id));
  const ExplicitFeatureDeletionTask& task(*task_ptr);
  assert(!task_id.empty());
  assert(boost::regex_match(task_id,boost::regex(".+:.+")));
  string feature_id(task.m_featureID);
  assert(!feature_id.empty());
  if(!boost::regex_match(feature_id,boost::regex(".+:.+"))) {
    cout << "\na very odd feature ID caught in GarbageCollector::explicitFeatureDeletionAdded: \"" 
	 << feature_id << "\""<<endl;
    abort();
  }
  m_deletionQueue.push_back(task_id);
  m_deletionQueue.push_back(feature_id);
}


void 
GarbageCollector::statusUpdated(const cast::cdl::WorkingMemoryChange& _wmc)
{
  if(!m_statusCache.get()) { // must allocate the cache when we know the address
    m_statusCache = 
      auto_ptr<CachedCASTData<BindingData::BinderStatus> >
      (new CachedCASTData<BindingData::BinderStatus>(*this,string(_wmc.m_address.m_id)));
  }
  // now assert there is always only one BinderStatus
  if(m_policeMode != DISABLED) {
    assert(m_statusCache->id() == string(_wmc.m_address.m_id));
  }
  try {
    while(!m_deletionQueue.empty() && 
	  (*m_statusCache)->m_stable) { // the cache loads the status again if needed
      deleteOneOnQueue();
    }
  } catch (DoesNotExistOnWMException& _e){
    cerr << "Caught this in GarbageCollector::statusUpdated: " << _e.what() << endl 
	 << "when loading BinderStatus on address: \"" << m_statusCache->id() << "\" (and/or deleting an element from the deletion queue)";
    abort();
  }
}

void 
GarbageCollector::reportIllegalSignal(const cast::cdl::WorkingMemoryChange& _wmc) 
{
  cerr << "\nIllegal WorkingMemoryChange signal received by GarbageCollector:\n";
  using cast::operator<<;
  //cast::operator<<(cerr,_wmc) << endl;
  cerr << _wmc << endl << endl;
  if(m_policeMode == STRICT)
    abort();
}


} // namespace Binding 
