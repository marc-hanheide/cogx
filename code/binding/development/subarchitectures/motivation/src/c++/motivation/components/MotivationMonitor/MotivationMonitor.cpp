#include "MotivationMonitor.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <motivation/idl/MotivationData.hh>


using namespace boost;
using namespace motivation::idl;

MotivationMonitor::MotivationMonitor(const string& _id) :
  WorkingMemoryAttachedComponent(_id),
  AbstractMonitor(_id),
  m_variableCount(0),
  m_handler(*this),
  m_variableListID(""),
  m_variableListLocked(false),
  m_statusCache(*this),
  m_statusID("")
{
  setReceiveXarchChangeNotifications(true); 
}

void
MotivationMonitor::start() {
  AbstractMonitor::start();
  m_sourceID = m_subarchitectureID;

  if(m_monitoredSAs.empty()) {
    log("no subarchitectures to monitor, doing nothing");
    return;
  }

  //we will listen for proxy overwrites as that means the proxies
  //must have been bound once already
  addChangeFilter(createChangeFilter<BindingData::BindingProxy>(cdl::OVERWRITE,
								"",
								"",
								getBindingSA(),
								cdl::ALL_SA),
		  new MemberFunctionChangeReceiver<MotivationMonitor>(this,
								      &MotivationMonitor::proxyOverwritten));

  addChangeFilter(createChangeFilter<BindingData::BindingProxy>(cdl::DELETE,
								"",
								"",
								getBindingSA(),
								cdl::ALL_SA),
		  new MemberFunctionChangeReceiver<MotivationMonitor>(this,
								      &MotivationMonitor::proxyDeleted));

  //we will listen for proxy deletions to clean everything up
  addChangeFilter(createChangeFilter<BindingData::BindingProxyDeletionTask>(cdl::ADD,
									    "",
									    "",
									    getBindingSA(),
									    cdl::ALL_SA),
		  new MemberFunctionChangeReceiver<MotivationMonitor>(this,
								      &MotivationMonitor::proxyScheduledForDeletion));


  //keep track of binder status for debugging
  addChangeFilter(createGlobalTypeFilter<BindingData::BinderStatus>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<MotivationMonitor>(this,
								      &MotivationMonitor::statusOverwritten));

  addChangeFilter(createGlobalTypeFilter<BindingData::BinderStatus>(cdl::ADD),
		  new MemberFunctionChangeReceiver<MotivationMonitor>(this,
								      &MotivationMonitor::statusOverwritten));

  addChangeFilter(createGlobalTypeFilter<motivation::idl::FeatureGenerationCompetence>(cdl::ADD),
		  new MemberFunctionChangeReceiver<MotivationMonitor>(this, 
								      &MotivationMonitor::generationCompetenceAdded));


}
  
bool 
MotivationMonitor::monitoredSource(const BindingFeatures::SourceID& _sid) const {
  string sourceID(_sid.m_sourceID);    
  return m_monitoredSAs.find(sourceID) != m_monitoredSAs.end();
}

void
MotivationMonitor::monitorProxy(const ProxyPtr& _proxy,
				const BindingFeatures::SourceID& _sid) {
  log("MotivationMonitor::monitorProxy: %s",_proxy->getID().c_str());
  m_monitoredProxies.insert(_proxy->getID());
  m_variableNames[_proxy->id()] = pair<string, string>("",string(_sid.m_sourceID));

  //we'll need to hang on here
  //LOOK OUT... can take this out if freeze when not testing
  startVarUpdate();
}
  
void 
MotivationMonitor::unmonitorProxy(const string& _proxyID) {
  log("MotivationMonitor::unmonitorProxy: %s",_proxyID.c_str());
  m_monitoredProxies.erase(_proxyID);   
  m_variableNames.erase(_proxyID);
}

void
MotivationMonitor::ignoreProxy(const ProxyPtr& _proxy) {
  log("MotivationMonitor::ignoreProxy: %s",_proxy->getID().c_str());
  m_ignoredProxies.insert(_proxy->getID());
}

void 
MotivationMonitor::unignoreProxy(const string& _proxyID) {
  log("MotivationMonitor::unignoreProxy: %s",_proxyID.c_str());
  m_ignoredProxies.erase(_proxyID);   
}


bool 
MotivationMonitor::previouslyProcessed(const string& _proxyID) const {
  return isMonitored(_proxyID) || isIgnored(_proxyID);
}
  
bool 
MotivationMonitor::isMonitored(const string& _proxyID) const {
  return m_monitoredProxies.find(_proxyID) != m_monitoredProxies.end();
}
  
bool 
MotivationMonitor::isIgnored(const string& _proxyID) const {
  return m_ignoredProxies.find(_proxyID) != m_ignoredProxies.end();
}


void 
MotivationMonitor::proxyDeleted(const cdl::WorkingMemoryChange& _wmc) {
  //state has changed
  stateChange();
  string proxyID(_wmc.m_address.m_id);
  deleteProxy(proxyID);
}

void 
MotivationMonitor::proxyScheduledForDeletion(const cdl::WorkingMemoryChange& _wmc) {
  //treat a deletion task as a deletion... less mess later

  //either way, a state change is afoot
  //stateChange();

  //need to lock up here because other things will come along later
  //
  //there will always be signals after this to clean up the lock
  lockVariableMappings();
  
  try {
    string proxyID(loadBindingDataFromWM<BindingData::BindingProxyDeletionTask>(_wmc.m_address)->m_proxyID);
    deleteProxy(proxyID);
  }
  catch (const DoesNotExistOnWMException & e) {
    log("MotivationMonitor::proxyScheduledForDeletion DoesNotExistOnWMException caught for BindingProxyDeletionTask: %s",
	e.what());
  }


}

void 
MotivationMonitor::stateChange() {
  
}
    
void 
MotivationMonitor::deleteProxy(const std::string& _proxyID) {

  log("MotivationMonitor::proxyDeleted: %s",_proxyID.c_str());

  //just clean up storage
  if(isMonitored(_proxyID)) {
    unmonitorProxy(_proxyID);
  }
  else if(isIgnored(_proxyID)) {
    unignoreProxy(_proxyID);
  }

  //now we should not know about it
  assert(!previouslyProcessed(_proxyID));
}

void 
MotivationMonitor::proxyOverwritten(const cdl::WorkingMemoryChange& _wmc) {
  //either way, a state change is afoot
  stateChange();

  string proxyID(_wmc.m_address.m_id);
  proxyOverwritten(proxyID);
}

void 
MotivationMonitor::proxyOverwritten(const std::string& proxyID) {

  try {

    log("MotivationMonitor::proxyOverwritten: %s",proxyID.c_str());
      
    if(previouslyProcessed(proxyID)) {
//         //we'll need to hang on here
//       //LOOK OUT... can take this out if freeze when not testing
//       if(isMonitored(proxyID)) {
// 	startVarUpdate();
//       }

      log("MotivationMonitor::proxyOverwritten seen before: %s",proxyID.c_str());
      return;
    }
      
    //load the proxy
    const ProxyPtr& proxy(m_proxyLocalCache.get(proxyID).getLocalPtr());
      
    //get its source id feature
    const BindingFeatures::SourceID& sid(proxy->getFeature<BindingFeatures::SourceID>());
      
    if(monitoredSource(sid)) {
      monitorProxy(proxy, sid);
      
    }
    else {
      ignoreProxy(proxy);
    }
      
      
  }
  catch (const DoesNotExistOnWMException & e) {
    log("MotivationMonitor::proxyOverwritten DoesNotExistOnWMException caught, trying delete: %s",e.what());
    deleteProxy(proxyID);
  }

 
 
}



void 
MotivationMonitor::statusOverwritten(const cdl::WorkingMemoryChange& _wmc) {

  if(m_statusID == "") {
    m_statusID = string(_wmc.m_address.m_id);
  }

  log("MotivationMonitor::statusOverwritten");
  

  //is the binder stable... 
  bool stable(binderStable());

  //if the binder is stable, but we're locked, then update vars and
  //free list
  if(stable && m_variableListLocked) {
    endVarUpdate();
    log("state: end");
  }
  //if the binder is not stable, but we're not locked, then lock up
  else if(!stable && !m_variableListLocked) {
    startVarUpdate();
    log("state: start");
  }
  else {
    log("state: n/a");
  }


}

void 
MotivationMonitor::endVarUpdate() {
  if(generateAndStoreMappings()) {
    unlockVariableMappings();    
  }
}

void 
MotivationMonitor::startVarUpdate() {
  lockVariableMappings();
}

void
MotivationMonitor::updateVariables(const UnionSet & _unions) {

  

  for(UnionSet::const_iterator u = _unions.begin();
      u != _unions.end(); ++u) {

    

    //proxies for this union
    const set<string> proxyIDs(u->second->proxyIDs());
      
    StringPairMap::const_iterator vn(m_variableNames.end());

    //see if any of the proxies already have a gensym
    for(set<string>::const_iterator p = proxyIDs.begin();
	p != proxyIDs.end(); ++p) {	
      vn = m_variableNames.find(*p);	
      //if we find a proxy that is associated with an existing
      //gensyn, then stop
      if(vn != m_variableNames.end() && vn->second.first != "") {
	break;
      }
      else {
	vn = m_variableNames.end();
      }
    }

    string varName;
    if(vn == m_variableNames.end()) {      
      varName = gensym();	
      debug("MotivationMonitor::updateVariables: new gensym %s",varName.c_str());      
    }
    else {
      varName = vn->second.first;
    }


    //now assign the var name to all the proxies in the union      
    for(set<string>::const_iterator p = proxyIDs.begin();
	p != proxyIDs.end(); ++p) {	
      vn = m_variableNames.find(*p);	
      //if this isn't in the var name map, then update
      if(vn == m_variableNames.end()) {
	log("MotivationMonitor::updateVariables: proxy %s missing",
	    p->c_str());
	//in this case there is a proxy that we haven't been signalled
	//about yet, so try to add it
	proxyOverwritten(*p);
	//and if it's a keeper, store a variable
	if(isMonitored(*p)) {
	  m_variableNames[*p].first = varName;
	}
      }
      else if(vn->second.first == "") {
	m_variableNames[*p].first = varName;
	log("MotivationMonitor::updateVariables: proxy %s mapped to var %s",
	      p->c_str(), m_variableNames[*p].first.c_str());
      }
      //if we already have 
      else {
	//don't let var names change... this will probably be wrong
	//very quickly

	//now letting them change
	log("MotivationMonitor::updateVariables: mapping proxy %s - mapped to var %s from %s",
	      p->c_str(), varName.c_str(), vn->second.first.c_str());	

	//assert(varName == vn->second.first);

	if(varName != vn->second.first) {
	  println("MotivationMonitor::updateVariables: renaming %s - mapped to var %s from %s",
		  p->c_str(), varName.c_str(), vn->second.first.c_str());	

	}
	m_variableNames[*p].first = varName;
      }
    }     
  }

}

void
MotivationMonitor::lockVariableMappings() {
  if(!m_variableListLocked && m_variableListID != "") {
    lockEntry(m_variableListID, cdl::LOCKED_ODR);
    log("locked variables");
    m_variableListLocked = true;
  }
}

void
MotivationMonitor::unlockVariableMappings() {
  if(m_variableListLocked && m_variableListID != "") {
    log("unlocked variables");
    unlockEntry(m_variableListID);
    m_variableListLocked = false;
  }
}


void 
MotivationMonitor::writeVariableMappings() {
  
  AddressVariableMappings * avm = new AddressVariableMappings();
  
  avm->m_mappings.length(m_variableNames.size());

  int i = 0;
  const char * proxyType(typeName<BindingData::BindingProxy>().c_str());
  for(StringPairMap::const_iterator vn = m_variableNames.begin();
      vn != m_variableNames.end(); ++vn) {
    //variable
    avm->m_mappings[i].m_variable = CORBA::string_dup(vn->second.first.c_str());
    //of proxy type
    avm->m_mappings[i].m_pointer.m_type = CORBA::string_dup(proxyType);
    //in the binding sa
    avm->m_mappings[i].m_pointer.m_address.m_subarchitecture = CORBA::string_dup(vn->second.second.c_str());
    //with this id
    avm->m_mappings[i].m_pointer.m_address.m_id = CORBA::string_dup(vn->first.c_str());    
    //and on to the next
    i++;
  }

  if(m_variableListID == "") {
    m_variableListID = newDataID();
    addToWorkingMemory(m_variableListID,avm);
  }
  else {
    overwriteWorkingMemory(m_variableListID,avm);
  }
  
}


void
MotivationMonitor::generationCompetenceAdded(const cast::cdl::WorkingMemoryChange& _wmc) {
  log("MotivationMonitor::generationCompetenceAdded");
  
  //just add the gen subarch to the variables
  shared_ptr<const FeatureGenerationCompetence> competence(getWorkingMemoryEntry<FeatureGenerationCompetence>(_wmc.m_address)->getData());	
  string subarch(competence->m_subarchitecture);
  
  string var = subarch;
  string::size_type dotpos = var.find("."); 
  while(dotpos != string::npos) {
    var.replace(dotpos,1,"_");
    dotpos = var.find("."); 
    //cout<<"var: "<<var<<endl;
  }
  
  m_variableNames[subarch].first = var;
  m_variableNames[subarch].second = subarch;
 
  log("stored subarch competence: " + subarch + " -> " + var);

}


bool
MotivationMonitor::generateAndStoreMappings() {

  if(m_monitoredProxies.size() == 0) {
    log("MotivationMonitor::generateAndStoreMappings: refusing to generate a state with no proxies");
    return false;
  }

  log("MotivationMonitor::generateAndStoreMappings: %d monitored, %d ignored",
      m_monitoredProxies.size(), m_ignoredProxies.size());
     
  try {

    //load data from fresh
    m_handler.purgeAllLoadedData();
    ProxySet proxies = m_handler.loadProxies(m_monitoredProxies);
    assert(proxies.size() == m_monitoredProxies.size());
    UnionSet unions = m_handler.extractUnionsFromProxies(proxies);

    log("MotivationMonitor::generateAndStoreMappings: %d proxies comprise %d unions",
	m_monitoredProxies.size(), unions.size());

//     ProxySet allProxies = m_handler.allProxiesFromWM();
//     unsigned int processed = m_monitoredProxies.size() + m_ignoredProxies.size();
//     if(allProxies.size() != processed) {
//       log("MotivationMonitor::generateAndStoreMappings: Not all proxies processed: %d processed !=  %d total",processed, allProxies.size());
//       return;
//     }

    //this makes sure all proxies map to a variable name
    updateVariables(unions);
      

    //store mappings on wm
    writeVariableMappings();
    return true;
  }
  catch (const DoesNotExistOnWMException & e ) {
    log("MotivationMonitor::generateAndStoreMappings: exception caught: %s",e.what());
    log("MotivationMonitor::generateAndStoreMappings: giving up on state generation");      
    //possibly delete?
  }
  catch (const BindingException & e ) {
    log("MotivationMonitor::generateAndStoreMappings: exception caught: %s",e.what());
    log("MotivationMonitor::generateAndStoreMappings: giving up on state generation");      
  }
  return false;

}

void 
MotivationMonitor::configure(map<string,string> & _config) {

  AbstractMonitor::configure(_config);

  setBindingSubarchID(getBindingSA());
  m_statusCache.setSubarchitectureID(getBindingSA());

  map<string,string>::const_iterator i = _config.find("--monitor");
  if(i != _config.end()) {
    vector<string> monitorList;
    SubarchitectureWorkingMemoryProtocol::tokenizeString(i->second,
							 monitorList,
							 ",");

    for(vector<string>::const_iterator j = monitorList.begin();
	j != monitorList.end(); ++j) {
      log("monitoring: %s", j->c_str());
      m_monitoredSAs.insert(*j);
    }
  }



}
void
MotivationMonitor::runComponent() {
  lockProcess();
  //do the first write with an empty list
  writeVariableMappings();
  //and lock up so we stall everything until the first state is available
  lockVariableMappings();

  unlockProcess();
}

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new MotivationMonitor(_id);
  }
}

