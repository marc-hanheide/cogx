#include "binding/utils/BindingUtils.hpp"
#include "Binder.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/utils/BindingUtils.hpp"
//#include "dot/DotUtils.hpp"
#include "binding/abstr/AbstractBinder.hpp"

#include <boost/algorithm/string.hpp>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach


using namespace std;
using namespace boost;
using namespace cast;


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::Binder(_id);
  }
}

namespace Binding {
using namespace std;
using namespace cast;
using namespace boost;
using namespace boost::assign;

Binder::Binder(const string &_id) : 
  
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id)
{ 
}

Binder::~Binder() {

}

void 
Binder::start() {

  AbstractBinder::start();

  addChangeFilter(createLocalTypeFilter<BindingData::BindingTask>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<Binder>(this,
							   &Binder::bindThisProxy));
    
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxyDeletionTask>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<Binder>(this,
							   &Binder::handleProxyDeletionTask));
  
  addChangeFilter(createLocalTypeFilter<BindingData::ProxyPorts>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<Binder>(this,
							   &Binder::updatePortsInUnion));

  addChangeFilter(createLocalTypeFilter<BindingData::ProxyPorts>(cdl::ADD),
		  new MemberFunctionChangeReceiver<Binder>(this,
							   &AbstractBinder::acquireEntry<BindingData::ProxyPorts>));
  
 

}

void 
Binder::configure(map<string, string>& _config)
{
  AbstractBinder::configure(_config);
} 

void
Binder::bindThisProxy(const cdl::WorkingMemoryChange & _wmc) {
  
  string bindingTaskID(_wmc.address.id);

  try {
    
    shared_ptr<const BindingData::BindingTask>
      bindingTask(loadBindingDataFromWM<BindingData::BindingTask>(bindingTaskID));
    
    
    string bestUnionsID(bindingTask->bestUnionsForProxyID);
    log("Going to bind based on this BestUnionsForProxy list: " + bestUnionsID);
    
    shared_ptr<const BindingData::BestUnionsForProxy>
      best(loadBindingDataFromWM<BindingData::BestUnionsForProxy>(bestUnionsID));
    
    assert(best->unionIDs.length() > 0);
    
    const string proxyID(best->proxyID);
    
    log("Proxy to bind: " + proxyID);  
    
    const LBindingProxy* proxy_ptr(maybeLoadProxy(proxyID));
    
    if(!proxy_ptr) {
      log("Proxy does not exist on WM: " + proxyID + " ignoring its scoring");
#warning SILLY TOKEN HACK, should not be necessary
      BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
      p->proxyID = CORBA::string_dup(proxyID.c_str());
      addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);
      // task is finished, delete it!
      deleteFromWorkingMemory(bindingTaskID);
      return;
    }
    const LBindingProxy& proxy(*proxy_ptr);
    
    if(proxy.type() == BindingData::RELATION)
      updateInports(proxy.rawOutPorts());

#warning disabled a failed sanity test, insane mode activated
/*    
    if(proxy->updates != best->proxyUpdatesWhenThisComputed) {
      log("WARNING!!! proxy->updates != best->proxyUpdatesWhenThisComputed\n");
      log("SKIPPING BINDING TASK DUE TO OUTDATED BEST-LIST");
      cout << "WARNING!!! proxy->updates != best->proxyUpdatesWhenThisComputed\n";
      cout << "SKIPPING BINDING TASK DUE TO OUTDATED BEST-LIST";
      deleteFromWorkingMemory(bindingTaskID);
      return;
    }
*/
    // always bind to whatever is the first on the list of best
    // bindings, even there may be multiple bindings that are the best,
    // but that's a disambiguation issue which the binder simply ignores
    const string unionID(best->unionIDs[0]);
    
    log("unionID to bind to: " + unionID);
    if(unionID != string(BindingData::NO_UNION) && !existsOnWorkingMemory(unionID)) {
      log(unionID + " does not exist on WM, this must be an old bestlist, do nothing");
      deleteFromWorkingMemory(bindingTaskID);
      return;
    }
    if(best->unionIDs.length() > 1)
      log("*************\nWarning: multiple instance bindings matched with proxy, this is a disambiguation issue...\n*************\n");
    // OK. ready to bind. 
    
    // if proxy is not bound, then just bind it
    if(prox2uni.find(proxyID) == prox2uni.end()) {
      log("not bound: " + proxyID + " let's bind it!");
      _bindProxy(proxy, unionID);
      // task is finished, delete it!
      deleteFromWorkingMemory(bindingTaskID);
      return;
    }
    
    // First, check if the proxy is already bound
    // to any of the bindings in the list of best. In that case do
    // nothing, as it would not increase the score at all. (this might
    // not be completely true if scoring could increase implicitly due
    // to relations elsewhere...)
    bool already_bound_to_one_of_the_best_unions(false);
    const string old_unionID(prox2uni[proxyID]);
    assert(old_unionID != "");
    
    
    for(unsigned int i = 0 ; 
	i < best->unionIDs.length() && !already_bound_to_one_of_the_best_unions; 
	++i) {
      if(old_unionID == string(best->unionIDs[i]) && old_unionID != string(BindingData::NO_UNION)) {
	already_bound_to_one_of_the_best_unions = true;
      }
    }
    log("already_bound_to_one_of_the_best_unions == %s", (already_bound_to_one_of_the_best_unions?"true":"false"));
    // task is finished, delete it!
    // deleteFromWorkingMemory(bindingTaskID);
    // return;
    
    
    // ok, it seems like the old and new binding IDs are not matching at
    // all. So we need to bind to the new unionID and update the old
    // unionID
    log("======= will bind proxy (" + proxyID + ") to new union (" + unionID +") and then remove it from old one (" + old_unionID + ")");
    
    if(!already_bound_to_one_of_the_best_unions) {
      //overwriteWorkingMemory(proxyID, new BindingData::BindingProxy(proxy.get()), cdl::BLOCKING);
      try{
	log("calling _bindProxy");
	_bindProxy(proxy, unionID);  
      }
      catch(DoesNotExistOnWMException& _e) {
	cerr << "thrown from _bindProxy:\n" << _e.what() 
	     << "should be caught in the function instead! aborting!" << endl;
	abort();
      }
      try {
	log("calling _removeProxyFromUnion");
	_removeProxyFromUnion(proxy, old_unionID);
      }
      catch(DoesNotExistOnWMException& _e) {
	cerr << "thrown from _removeProxyFromUnion:\n" << _e.what() 
	     << "should be caught in the function instead! aborting!" << endl;
	abort();
      }
    } else {
      try {
	log("calling _updateProxyInUnion");
	_updateProxyInUnion(proxy, unionID);
      }
      catch(DoesNotExistOnWMException& _e) {
	cerr << "thrown from _removeProxyFromUnion:\n" << _e.what() 
	     << "should be caught in the function instead! aborting!" << endl;
	abort();
      }
    }
    
    // task is finished, delete it!
    deleteFromWorkingMemory(bindingTaskID);
  }
  catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in BindingScorer (NO LONGER ACCEPTED): " + string(_e.what()));
    // task is obsolete since proxy was deleted (most likely)...
    //    deleteFromWorkingMemory(bindingTaskID);
    abort();
  }
}

void 
Binder::updateInports(const BindingData::ProxyPorts& _outports)
{
  for(unsigned int i = 0 ; i < _outports.ports.length() ; ++i) {
    const BindingData::ProxyPort& the_outport(_outports.ports[i]);    
    string proxyID(the_outport.proxyID);
    string ownerProxyID(the_outport.ownerProxyID);
    const LBindingProxy* proxy = maybeLoadProxy(proxyID);
    if(proxy) {
      log("updating inports of " + proxyID);
      std::set<BindingData::ProxyPort, proxyPortLess> new_inports;
      const string inportsID((*proxy)->inPortsID);
      try {
	static cast::CASTDataCache<BindingData::ProxyPorts> inportscache(*this);
	const BindingData::ProxyPorts& old_inports(inportscache[inportsID]);
	// 1st copy old relevant
	for(unsigned int j; j < old_inports.ports.length() ; ++j) {
	  const BindingData::ProxyPort& the_inport(old_inports.ports[j]);
	  if(string(the_inport.proxyID) !=
	     ownerProxyID)
	    new_inports.insert(the_inport);
	}
	BindingData::ProxyPort new_port;
	new_port.type = the_outport.type;
	new_port.label = CORBA::string_dup(the_outport.label);
	new_port.proxyID = CORBA::string_dup(the_outport.ownerProxyID);
	//cout << "new_port.proxyID " << new_port.proxyID << endl;
	new_port.ownerProxyID = CORBA::string_dup(the_outport.proxyID);
	new_inports.insert(new_port);
	BindingData::ProxyPorts* _new_inports = new BindingData::ProxyPorts(old_inports);
	_new_inports->ports.length(new_inports.size());
	unsigned int k = 0;
	foreach(const BindingData::ProxyPort p, new_inports) {
	  _new_inports->ports[k++] = p;
	  assert(string(p.proxyID) != "");
	  assert(string(p.ownerProxyID) != "");
	  assert(string(p.label) != "");
	}
	overwriteWorkingMemory(inportsID, 
			       _new_inports,
			       cast::cdl::BLOCKING);
      }
      catch(const DoesNotExistOnWMException& _e) {
	if(string(_e.address().id) == inportsID)
	  log(string("missing inport: ") + _e.what());
	else
	  throw _e;
      }
    }
  }

  /*
    
  for(unsigned int i = 0 ; i < _ports.ports.length() ; ++i) {
  string proxyID(_ports.ports[i].proxyID);
  log("updating inports of " + proxyID);
  try {
  map<string,string>::const_iterator inportsID_itr(proxyID2inportsID.find(proxyID));
  string inportsID;
  if(inportsID_itr == proxyID2inportsID.end()) {
  shared_ptr<const CASTData<BindingData::BindingProxy> > data_ptr(getWorkingMemoryEntry<BindingData::BindingProxy>(proxyID,bindingSA));
  shared_ptr<const BindingData::BindingProxy> froproxy(data_ptr->getData());
  inportsID = froproxy->inPortsID;
  proxyID2inportsID[proxyID] = inportsID;
  } else {
  inportsID = inportsID_itr->second;
  }
  log("inportsID: " + inportsID);
  
  //      lockEntry(inportsID, bindingSA, cast::cdl::LOCKED_ODR);
  shared_ptr<const CASTData<BindingData::ProxyPorts> > 
  inportsData = 
  getWorkingMemoryEntry<BindingData::ProxyPorts>(inportsID, bindingSA);
  shared_ptr<const BindingData::ProxyPorts> inports = inportsData->getData();
  BindingData::ProxyPorts* new_inports = new BindingData::ProxyPorts(*inports);
  
  //ProxyPort[] new_inports = new ProxyPort[inports.ports.length + 1];
  //System.arraycopy(inports.ports, 0, new_inports, 0, inports.ports.length);
  //new_inports[inports.ports.length] = new ProxyPort(_portType,"",_relationID);
  new_inports->ports.length(inports->ports.length() + 1);
  BindingData::ProxyPort new_port;
  new_port.type = _ports.ports[i].type;
  new_port.label = CORBA::string_dup(_ports.ports[i].label);
  new_port.proxyID = CORBA::string_dup(_ports.ports[i].ownerProxyID);
  //cout << "new_port.proxyID " << new_port.proxyID << endl;
  new_port.ownerProxyID = CORBA::string_dup(_ports.ports[i].proxyID);
  
  new_inports->ports[inports->ports.length()] = new_port;
  
  overwriteWorkingMemory(inportsID, 
  bindingSA,
  //BindingLocalOntology::PROXY_PORTS_TYPE,
  new_inports,
  BLOCKING);
  //insert the proxy to the unbound list.
  unboundProxyAddresses.insert(proxyID);
  } catch(const DoesNotExistOnWMException& _e) {
  log("could not update inports of " + proxyID + " : " + _e.what());
  }
  }
  */
}


void 
Binder::_createNewUnion(const LBindingProxy& _proxy) {
  
  //shared_ptr<const BindingData::BindingProxy>
  //proxy(loadBindingDataFromWM<BindingData::BindingProxy>(_proxyID));

//  const LBindingProxy& proxy(proxyLocalCache[_proxyID]);    

  //const FeatureSet fset(toFeatureSet(proxy.proxyFeatures));
  const FeatureSet& fset(_proxy.featureSet());
  
  const string& proxyID(_proxy.id());
  
  // create an instance binding and store it...
  BindingData::BindingUnion* ib = new BindingData::BindingUnion();
  
  ib->proxyIDs.length(1);
  ib->proxyIDs[0] = CORBA::string_dup(proxyID.c_str());
  ib->featureSignatures.length(1);
  ib->featureSignatures[0] = CORBA::string_dup(_proxy->featureSignature);
  ib->updates = 0;
  ib->type = _proxy->type;
  
  ib->outPorts = _proxy.rawOutPorts();
  ib->inPorts = _proxy.inPorts();
  
  unsigned int i = 0;
  for(FeatureSet::const_iterator feature_type_itr = fset.begin(); 
      feature_type_itr != fset.end() ; 
      ++feature_type_itr) {
    ib->unionFeatures.length(ib->unionFeatures.length() + 
				 feature_type_itr->second.size());
    for(OneTypeOfFeatures::const_iterator feature_itr = feature_type_itr->second.begin() ;
	feature_itr != feature_type_itr->second.end() ; 
	++i,++feature_itr) {
      assert(i < ib->unionFeatures.length());      
      ib->unionFeatures[i].address = CORBA::string_dup((*feature_itr)->featureID().c_str());
      ib->unionFeatures[i].type =    CORBA::string_dup((*feature_itr)->name().c_str());
      ib->unionFeatures[i].immediateProxyID =    CORBA::string_dup((*feature_itr)->immediateProxyID().c_str());
      //log(lexical_cast<string>(i) + " : " + lexical_cast<string>(ib->unionFeatures.length()));
    }
  }
  string unionID = newDataID();
  
  _checkRelationsAndTriggerRescoring(ib->inPorts,ib->outPorts);
  
  addToWorkingMemory(unionID,
		     //BindingLocalOntology::BINDING_UNION_TYPE, 
		     ib, 
		     cdl::BLOCKING);
  lockEntry(unionID,cast::cdl::LOCKED_O);
  _updateProxyPointers(_proxy, unionID, *ib);
  log(string("New union ( ") + unionID + ") created for proxy " + proxyID);
  
  prox2uni[proxyID] = unionID;
  uni2prox[unionID].insert(proxyID);

}


void
Binder::_addProxyFeaturesToUnionFeatures(const BindingData::FeaturePointers& _proxyFeatures,
					       BindingData::FeaturePointers& _unionFeatures) const {
  
    unsigned int ib_feature_length = _unionFeatures.length();
  
  // allocate space
  //cout << "in binding:\n" << toString(ib->unionFeatures);
  //cout << "in proxy:\n" << toString(proxy.proxyFeatures);
  _unionFeatures.length(_unionFeatures.length() + _proxyFeatures.length());
      
  // add new features from proxy
  for(unsigned int j = 0 ; j < _proxyFeatures.length() ; ++j) {
    _unionFeatures[ib_feature_length + j] = _proxyFeatures[j];
  }
}

void
Binder::_addProxyPortsToUnionPorts(const LBindingProxy& proxy, BindingData::BindingUnion& new_union) const
{
  {
    // copy the inports
    set<BindingData::ProxyPort, proxyPortLess> ports;
    insert_iterator<set<BindingData::ProxyPort, proxyPortLess> > inserter = std::inserter(ports,ports.begin());
    for(unsigned int i = 0; i < new_union.inPorts.ports.length() ; ++i) {
      inserter = new_union.inPorts.ports[i];
    }
    for(unsigned int i = 0; i < proxy.inPorts().ports.length() ; ++i) {
      inserter = proxy.inPorts().ports[i];
    }
    unsigned int j = 0;
    new_union.inPorts.ports.length(ports.size());
    foreach(const BindingData::ProxyPort& port, ports) {
      new_union.inPorts.ports[j++] = port;
    }
  }
  {
    // copy the outports
    set<BindingData::ProxyPort, proxyPortLess> ports;
    insert_iterator<set<BindingData::ProxyPort, proxyPortLess> > inserter = std::inserter(ports,ports.begin());
    for(unsigned int i = 0; i < new_union.outPorts.ports.length() ; ++i) {
      inserter = new_union.outPorts.ports[i];
    }
    for(unsigned int i = 0; i < proxy.rawOutPorts().ports.length() ; ++i) {
      inserter = proxy.rawOutPorts().ports[i];
    }
    unsigned int j = 0;
    new_union.outPorts.ports.length(ports.size());
    foreach(const BindingData::ProxyPort& port, ports) {
      new_union.outPorts.ports[j++] = port;
    }
  }
  
  
  /*  {
      const BindingData::ProxyPorts& proxy_inports(proxy.inPorts());	      
      unsigned int in_length = new_union.inPorts.ports.length();
      new_union.inPorts.ports.length(in_length + proxy_inports.ports.length());
      for(unsigned int j = 0 ; j < proxy_inports.ports.length() ; ++j) {
      new_union.inPorts.ports[in_length + j] = proxy_inports.ports[j];
      }
      }
      {
      const BindingData::ProxyPorts& proxy_outports(proxy.rawOutPorts());	      
      unsigned int out_length = new_union.outPorts.ports.length();
      new_union.outPorts.ports.length(out_length + proxy_outports.ports.length());
      for(unsigned int j = 0 ; j < proxy_outports.ports.length() ; ++j) {
      new_union.outPorts.ports[out_length + j] = proxy_outports.ports[j];
      }
      }
  */
}


void
Binder::_bindProxy(const LBindingProxy& _proxy, 
		   string _unionID) 
{
  
  const string& proxyID(_proxy.id());
  log(string("about to bind proxy ") + proxyID + " with union "+ _unionID);
  
  // if the binding is not existing, we'll create a new one instead of
  // binding to an existing one
  if(_unionID == string(BindingData::NO_UNION)) {
    _createNewUnion(_proxy);    
    return;
  }
  
  // look up old binding
  //  shared_ptr<const BindingData::BindingUnion>
    //old_union(loadBindingDataFromWM<BindingData::BindingUnion>(_unionID));
  //  old_union(unionCache.getPtr(_unionID));
  
  const LBindingUnion& old_union(unionLocalCache[_unionID]);

  BindingData::BindingUnion* new_union = new BindingData::BindingUnion(old_union.get());
  

  // now copy all features, there will be duplicates if features
  // from different subarchitectures are identical. Perhaps this
  // could be dealt with in a more optimal way. But it shouldn't
  // create a large overhead.
  unsigned int ib_feature_length = old_union->unionFeatures.length();
  // allocate space
  //cout << "in binding:\n" << toString(ib->unionFeatures);
  //cout << "in _proxy:\n" << toString(_proxy->proxyFeatures);
  new_union->unionFeatures.length(old_union->unionFeatures.length() + _proxy->proxyFeatures.length());
  
  // copy old features
  for(unsigned int j = 0 ; j < old_union->unionFeatures.length() ; ++j) {
    new_union->unionFeatures[j] = old_union->unionFeatures[j];
  }
    
  // add new features from proxy
  for(unsigned int j = 0 ; j < _proxy->proxyFeatures.length() ; ++j) {
    new_union->unionFeatures[ib_feature_length + j] = _proxy->proxyFeatures[j];
  }
  // copy the ports

  log("copying ports when updating a union");  
  new_union->inPorts = old_union->inPorts;
  new_union->outPorts = old_union->outPorts;
  log("still copying ports when updating a union");  
  _addProxyPortsToUnionPorts(_proxy,*new_union);
  log("done copying ports when updating a union");  

  // copy the associated proxy IDs
  new_union->proxyIDs.length(old_union->proxyIDs.length() + 1);
  for(unsigned int j = 0 ; j < old_union->proxyIDs.length() ; ++j) {
    new_union->proxyIDs[j] = old_union->proxyIDs[j];
  }
  // associate to the new proxy
  new_union->proxyIDs[new_union->proxyIDs.length() - 1] = 
    CORBA::string_dup(cdl::WorkingMemoryID(proxyID.c_str()));      

  // copy the associated proxy feature signatures
  new_union->featureSignatures.length(old_union->featureSignatures.length() + 1);
  for(unsigned int j = 0 ; j < old_union->featureSignatures.length() ; ++j) {
    new_union->featureSignatures[j] = old_union->featureSignatures[j];
  }
  new_union->featureSignatures[new_union->featureSignatures.length() - 1] = 
    CORBA::string_dup(_proxy->featureSignature);      
  //insanity check
  assert(new_union->proxyIDs.length() == new_union->featureSignatures.length());
  
  new_union->updates ++;

  _checkRelationsAndTriggerRescoring(new_union->inPorts,new_union->outPorts);

  // store on WM
//#warning should have a lock here...
//  loadBindingDataFromWM<BindingData::BindingUnion>(_unionID);
  overwriteWorkingMemory(_unionID, 
			 //BindingLocalOntology::BINDING_UNION_TYPE, 
			 new_union, 
			 cdl::BLOCKING);
  
  try{
//    for(unsigned int i = 0; i < new_union->proxyIDs.length() ; ++i)
    //     lockEntry(string(new_union->proxyIDs[i]), cast::cdl::LOCKED_ODR);
    for(unsigned int i = 0; i < new_union->proxyIDs.length() ; ++i)
      _updateProxyPointers(proxyLocalCache[string(new_union->proxyIDs[i])], _unionID, *new_union);
 //   for(unsigned int i = 0; i < new_union->proxyIDs.length() ; ++i)
 //     unlockEntry(string(new_union->proxyIDs[i]));
    
  } catch(...) {
    cerr << "don't wanna catch nothing here!" << endl;
    abort();
  }
  prox2uni[proxyID] = _unionID;
  uni2prox[_unionID].insert(proxyID);  
  
  log(string("bound proxy ") + proxyID + " with union "+ _unionID);
  
}

void
Binder::_updateProxyInUnion(const LBindingProxy& _proxy, 
			    const string& _unionID) 
{

  const string& proxyID(_proxy.id());
  log(string("updating proxy ") + proxyID + " in union "+ _unionID);
  
  // if the binding is not existing, we'll create a new one instead of
  // binding to an existing one
  assert(_unionID != string(BindingData::NO_UNION));
  
  // look up old binding
  //  shared_ptr<const BindingData::BindingUnion>
    //old_union(loadBindingDataFromWM<BindingData::BindingUnion>(_unionID));
  //  old_union(unionCache.getPtr(_unionID));
  
  const LBindingUnion& old_union(unionLocalCache[_unionID]);

  BindingData::BindingUnion* new_union = new BindingData::BindingUnion(old_union.get());

  
  // now copy all features, there will be duplicates if features
  // from different subarchitectures are identical. Perhaps this
  // could be dealt with in a more optimal way. But it shouldn't
  // create a large overhead.
  new_union->unionFeatures.length(0);
  // copy old features (except the one from updated proxy)
  unsigned int i = 0;
  for(unsigned int j = 0 ; j < old_union->unionFeatures.length() ; ++j) {
    if(string(old_union->unionFeatures[j].immediateProxyID) != proxyID) {
      new_union->unionFeatures.length(i+1);
      new_union->unionFeatures[i++] = old_union->unionFeatures[j];
    }
  }
  // add new features from proxy
  for(unsigned int j = 0 ; j < _proxy->proxyFeatures.length() ; ++j) {
    new_union->unionFeatures.length(i+1);
    new_union->unionFeatures[i++] = _proxy->proxyFeatures[j];
  }
  // copy the ports

#warning updating of unions are not really doing a good job on the ports...
  new_union->inPorts = old_union->inPorts;
  new_union->outPorts = old_union->outPorts;
  _addProxyPortsToUnionPorts(_proxy,*new_union);

  // copy the associated proxy IDs
  /*  new_union->proxyIDs.length(old_union->proxyIDs.length() + 1);
      for(unsigned int j = 0 ; j < old_union->proxyIDs.length() ; ++j) {
      new_union->proxyIDs[j] = old_union->proxyIDs[j];
      }
      
      // associate to the new proxy
      new_union->proxyIDs[new_union->proxyIDs.length() - 1] = 
      CORBA::string_dup(cdl::WorkingMemoryID(proxyID.c_str()));      
  */  
  
  new_union->updates ++;

  _checkRelationsAndTriggerRescoring(new_union->inPorts,new_union->outPorts);

  // store on WM
  overwriteWorkingMemory(_unionID, 
			 //BindingLocalOntology::BINDING_UNION_TYPE, 
			 new_union, 
			 cdl::BLOCKING);
  
  //_updateProxyPointers(_proxy, _unionID);
  
  //prox2uni[proxyID] = _unionID;
  //uni2prox[_unionID].insert(proxyID);  
  
  log(string("rebound proxy ") + proxyID + " with binding "+ _unionID);
  
}

void
Binder::_updateProxyPointers(const LBindingProxy& _proxy, 
			     const string& _unionID,
			     const BindingData::BindingUnion& _union)
{
  BindingData::BindingProxy* new_proxy = new BindingData::BindingProxy(_proxy.get());
  
  new_proxy->unionID = CORBA::string_dup(_unionID.c_str());
  new_proxy->proxyIDs.length(_union.proxyIDs.length());
  for(unsigned int i = 0; i < _union.proxyIDs.length() ; ++i) {
    new_proxy->proxyIDs[i] = CORBA::string_dup(string(_union.proxyIDs[i]).c_str());
  }

  //new_proxy->bestUnionsForProxyID = CORBA::string_dup(_bestUnionsID.c_str());
  
  new_proxy->bindingCount++;
  
  //assert(new_proxy->proxyState != BindingData::BOUND);
  
  new_proxy->proxyState = BindingData::BOUND;

  // to pass the consistency check, the data must be loaded before
  // it's written. It's not strictly necessary for any other purpose
  // in this particular case.
  getWorkingMemoryEntry<BindingData::BindingProxy>(_proxy.id()); 
  
  overwriteWorkingMemory(_proxy.id(), 
			 //BindingLocalOntology::BINDING_PROXY_TYPE, 
			 new_proxy,
			 cdl::BLOCKING);

#warning SILLY TOKEN HACK, should not be necessary
  BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
  p->proxyID = CORBA::string_dup(_proxy.id().c_str());
  addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);

}

void 
Binder::handleProxyDeletionTask(const cdl::WorkingMemoryChange & _wmc) 
{

  shared_ptr<const BindingData::BindingProxyDeletionTask>
    deletion_task(loadBindingDataFromWM<BindingData::BindingProxyDeletionTask>(_wmc));
  // we don't need to keep the task after it has been read
  deleteFromWorkingMemory(string(_wmc.address.id));

  const string proxyID(deletion_task->proxyID);
  //shared_ptr<const BindingData::BindingProxy>
    //proxy(loadBindingDataFromWM<BindingData::BindingProxy>(proxyID));
    //proxy(proxyCache.getPtr(proxyID));
  
  
  const LBindingProxy* proxy_ptr = NULL;
  try {
    proxy_ptr = &(proxyLocalCache[proxyID]);
  } catch(const DoesNotExistOnWMException& _e) {
    cout << "\nCaught this in Binder::handleProxyDeletionTask: " << _e.what() 
	 << "\nPossible cause: did you remove the proxy twice? Or did you remove it by any other means than AbstractMonitor::deleteExistingProxy(...)?\n" << endl;
    abort();
  }
  assert(proxy_ptr);
  const LBindingProxy& proxy(*proxy_ptr);

  log("will attempt to remove proxy: " + proxyID);
  
  map<string,string>::const_iterator c2b = prox2uni.find(proxyID);
  if(c2b == prox2uni.end()) {
    log("Binder thinks this proxy was never bound, so it will simply just remove it without updating any binding");
  } else {
    string unionID(c2b->second);
    log("Binder will remove proxy " + proxyID + " from binding " + unionID);
    try{
      _removeProxyFromUnion(proxy, unionID);
    }
    catch(DoesNotExistOnWMException& _e) {
      cerr << "thrown from _removeProxyFromUnion:\n" << _e.what() 
	   << "\nshould be caught in the function instead! aborting!" << endl;
      abort();
    }
  }
  
  if(proxy.type() == BindingData::RELATION) {
    removeInports(proxy.rawOutPorts());
  }
  
  // get rid of the proxy for good 

  
  _checkRelationsAndTriggerRescoring(proxy.inPorts(),proxy.rawOutPorts());
  // nah: made this sync so there's no danger of us rereading a
  // proxy from wm that does not exist
  deleteFromWorkingMemory(proxyID, cdl::BLOCKING);


  // remove locally
  prox2uni.erase(proxyID);
  
  BindingData::DeletedBindingProxy* deleted_proxy = new BindingData::DeletedBindingProxy;
  deleted_proxy->deletedProxy = proxy.get();
  deleted_proxy->deletedProxyID = CORBA::string_dup(proxyID.c_str());
  const string newID(newDataID()); 
  log("Adding \"deleted\" proxy " + newID + " to replace the proxy " + proxyID);

  addToWorkingMemory(newID,
		     //BindingLocalOntology::BINDING_PROXY_SAFE_FOR_DELETION_TYPE,
		     deleted_proxy);
  
  log("Added \"deleted\" proxy " + newID + " to replace the proxy " + proxyID);
  
}

void 
Binder::updatePortsInUnion(const cdl::WorkingMemoryChange & _wmc) {

  //begin nah change
  /*  if(strcmp(_wmc.src,getProcessIdentifier().c_str()) == 0) {
      log("Binder::updatePortsInUnion: ignoring own changes to %s", string(_wmc.address.id).c_str());
      return;
      }*/
  //end nah change

  shared_ptr<const BindingData::ProxyPorts>
    updated_inports(loadBindingDataFromWM<BindingData::ProxyPorts>(_wmc));
  
  //begin nah change
  /*  //on deletion of a proxy, could this legitimately be 0?
      if(updated_inports->ports.length() == 0) {
      log("Binder::updatePortsInUnion: %s -> no ports to update", string(_wmc.address.id).c_str());
      return;
      }*/
  //end nah change

  //  assert(updated_inports->ports.length() > 0);  

  string updated_proxyID(updated_inports->ownerProxyID);

  try {
    const LBindingProxy& proxy(proxyLocalCache[updated_proxyID]);
    
    string unionID(proxy->unionID);
    if(unionID == "") { // i.e. not yet a bound proxy 
      log("Binder::updatePortsInUnion: the inport of an unbound proxy updated, do nothing");
      return;
    }
    
    const LBindingUnion& binding_union(unionLocalCache[unionID]);
    // create a new union that will hold the new union
    BindingData::BindingUnion* new_union = 
      new BindingData::BindingUnion(binding_union.get()); 
    // remove old inports
    new_union->inPorts.ports.length(0); 
    
    //  new_union->inPorts.ports = 34; 
    //  new_union->inPorts.ports = BindingData::ProxyPorts::_ports_seq();
    
    set<BindingData::ProxyPort, proxyPortLess> ports;
    insert_iterator<set<BindingData::ProxyPort, proxyPortLess> > inserter = std::inserter(ports,ports.begin());
    for(unsigned int i = 0; i < binding_union.inPorts().ports.length() ; ++i) {
      //      if(string(binding_union.inPorts().ownerProxyID) != updated_proxyID)
      inserter = binding_union.inPorts().ports[i];
    }
    for(unsigned int i = 0; i < updated_inports->ports.length() ; ++i) {
      inserter = updated_inports->ports[i];
      assert(string(updated_inports->ports[i].ownerProxyID) == 
	     string(updated_inports->ports[0].ownerProxyID));
      assert(string(updated_inports->ports[i].ownerProxyID) == 
	     string(updated_inports->ownerProxyID));
    }
    unsigned int j = 0;
    new_union->inPorts.ports.length(ports.size());
    foreach(const BindingData::ProxyPort& port, ports) {
      new_union->inPorts.ports[j++] = port;
    }
    /*  unsigned int j = 0;
	for(unsigned int i = 0; i < binding_union.inPorts().ports.length() ; ++i) {
	if(string(binding_union.inPorts().ports[i].ownerProxyID) != updated_proxyID) { // i.e. no copy of old proxy IDs
	new_union->inPorts.ports.length(j+1);
	new_union->inPorts.ports[j++] = binding_union.inPorts().ports[i];
	}
	}
	for(unsigned int i = 0; i < updated_inports->ports.length() ; ++i) {
	new_union->inPorts.ports.length(j+1);
	new_union->inPorts.ports[j++] = updated_inports->ports[i];
	assert(string(updated_inports->ports[i].ownerProxyID) == string(updated_inports->ports[0].ownerProxyID));
	}
    */
    log("Binder::updatePortsInUnion: the inport of union " + unionID + " updated");
    log("number of inports is now: " + lexical_cast<string>(new_union->inPorts.ports.length()));
    
    //  cout << "OVERWRITING THE UNION NOW, EH!" << endl;
    
    overwriteWorkingMemory(unionID,
			   //BindingLocalOntology::BINDING_UNION_TYPE,
			   new_union//,cdl::BLOCKING
			   ); 
  } catch (const DoesNotExistOnWMException& _e) {
    log("proxy deleted before the port update in union could be executed: %s" , _e.what());
  }
}

void
Binder::_removeProxyFromUnion(const LBindingProxy& _proxy, 
			      const string& _unionID)
{ 
  
  const LBindingUnion& binding_union(unionLocalCache[_unionID]);
  const string proxyID(_proxy.id());
  log(string("Removing proxy " + proxyID + " from union " + _unionID));

  assert(binding_union->proxyIDs.length() > 0);

  if(binding_union->proxyIDs.length() == 1) { // last proxy of this union
    log(string("Deleting union ") + _unionID + " since proxy " + proxyID + " was the last proxy.");

    deleteFromWorkingMemory(_unionID, cdl::BLOCKING); //nah: sync write to
    //make sure it's really
    //gone before we do
    //anything else

    if(prox2uni[proxyID] == _unionID) {
      prox2uni.erase(proxyID);
    }
    uni2prox.erase(_unionID);
    return;
  } 
  
  // copy all proxies except this one
  log(string("Removing proxy ") + proxyID + " from " + _unionID);
  
  // create a new binder that will hold the results
  BindingData::BindingUnion* new_union = 
    new BindingData::BindingUnion(binding_union.get()); 
  
  // remove all old features
  new_union->unionFeatures.length(0); 
  // remove all old ports
  new_union->inPorts.ports.length(0); 
  new_union->outPorts.ports.length(0); 
  
  assert(binding_union->proxyIDs.length() > 0);

  // number of proxies are 1 fewer
  new_union->proxyIDs.length(binding_union->proxyIDs.length() - 1);
  new_union->featureSignatures.length(binding_union->featureSignatures.length() - 1);
  assert(new_union->proxyIDs.length() == new_union->featureSignatures.length());
  
  unsigned int j = 0;
  for(unsigned int i = 0 ; i < binding_union->proxyIDs.length() ; ++i) {
    if(string(binding_union->proxyIDs[i]) != proxyID) {
      assert(j < new_union->proxyIDs.length());
      new_union->proxyIDs[j] = CORBA::string_dup((binding_union->proxyIDs)[i]);
      new_union->featureSignatures[j] = 
	CORBA::string_dup((binding_union->featureSignatures)[i]);
      //shared_ptr<const BindingData::BindingProxy>
      //proxy(loadBindingDataFromWM<BindingData::BindingProxy>(binding_union->proxyIDs[i]));	
      try {
	const LBindingProxy& proxy(proxyLocalCache[string(binding_union->proxyIDs[i])]);    
	_addProxyFeaturesToUnionFeatures(proxy->proxyFeatures,new_union->unionFeatures);
	_addProxyPortsToUnionPorts(proxy,*new_union);
	j++; 
      } catch(const DoesNotExistOnWMException& _e) {
	log("Proxy does not exist. Error in Binder.cpp");
	abort();
      }
    }
  }

  

  new_union->updates ++;
  _checkRelationsAndTriggerRescoring(new_union->inPorts,new_union->outPorts);
  overwriteWorkingMemory(_unionID,
			 //BindingLocalOntology::BINDING_UNION_TYPE,
			 new_union//,cdl::BLOCKING
			 ); 
  log(_unionID + " updated after proxy removal");
  
  if(prox2uni[proxyID] == _unionID) {
    prox2uni.erase(proxyID);
  }
  uni2prox[_unionID].erase(proxyID);

}


void
Binder::_checkRelationsAndTriggerRescoring(const BindingData::ProxyPorts& _inports, 
					   const BindingData::ProxyPorts& _outports) 
{
//#warning testing to disable _checkRelationsAndTriggerRescoring
#warning testing to enable _checkRelationsAndTriggerRescoring
  set<string> ported_proxies;
  for(unsigned int i = 0 ; i < _inports.ports.length() ; ++i) {
    ported_proxies.insert(string(_inports.ports[i].proxyID));
  }
  for(unsigned int i = 0 ; i < _outports.ports.length() ; ++i) {
    ported_proxies.insert(string(_outports.ports[i].proxyID));
  }
  if(!ported_proxies.empty()) {
    BindingData::BindTheseProxies* bindThese = new BindingData::BindTheseProxies();
    bindThese->proxyIDs.length(ported_proxies.size());
    set<string>::const_iterator proxy_itr = ported_proxies.begin();
    for(unsigned int i = 0 ; i < bindThese->proxyIDs.length() ; ++i,++proxy_itr) {
      bindThese->proxyIDs[i] = CORBA::string_dup(proxy_itr->c_str());
      //const LBindingProxy& proxy(proxyLocalCache[string(bindThese->proxyIDs[i])]);
      const LBindingProxy* proxy_ptr = maybeLoadProxy(string(bindThese->proxyIDs[i]));
      
      if(proxy_ptr && proxy_ptr->proxyState() == BindingData::BOUND) {
	changeProxyState(*proxy_ptr,
			 BindingData::REPROCESSED);
      }
    }
    addToWorkingMemory(newDataID(), 
		       subarchitectureID, 
		       //BindingLocalOntology::BIND_THESE_PROXIES_TYPE, 
		       bindThese, 
		       cdl::BLOCKING);
  }

}

void Binder::runComponent() {
}

void Binder::taskAdopted(const string &_taskID) {
    
}

void Binder::taskRejected(const string &_taskID) {
}


void 
Binder::removeInports(const BindingData::ProxyPorts& _ports)
{
  set<string> proxies;
  for(unsigned int i = 0 ; i < _ports.ports.length() ; ++i) {
    string proxyID(_ports.ports[i].proxyID);
    const LBindingProxy* proxy(maybeLoadProxy(proxyID));
    log("removing inports of related-to proxy %d, %s ",i,proxyID.c_str());
    if(proxy) {
      try {
	
	string inportsID((*proxy)->inPortsID);	
	shared_ptr<const CASTData<BindingData::ProxyPorts> > inportsData = getWorkingMemoryEntry<BindingData::ProxyPorts>(inportsID);
	shared_ptr<const BindingData::ProxyPorts> inports = inportsData->getData();
	PortMap ppp;

	//begin nah change -- this was one of the failure points
	//log("length of inports on the relation proxy to be deleted: %d", _ports.ports.length());
	//log("length of inports on the proxy being updated after relation deletion: %d", inports->ports.length());
	//assert(_ports.ports.length() == inports->ports.length());
	//end nah change
	
	// copy the inports, except for the
	for(unsigned int j = 0; j < inports->ports.length() ; ++j) {
	  if(string(inports->ports[j].ownerProxyID) == 
	     //begin nah change j to i
	     string(_ports.ports[i].proxyID) || 
	     //end nah change
	     string(inports->ports[j].proxyID) == 
	     //begin nah change j to i
	     string(_ports.ports[i].ownerProxyID)
	     //end nah change
	     ) {
	    proxies.insert(proxyID); // since something was now not copied...x
	  } else { // copy the old one
	    ppp[string(inports->ports[j].label)].insert(inports->ports[j]);
	  }
	  cout << i << " " << j << endl;
	}
	BindingData::ProxyPorts* new_inports = new BindingData::ProxyPorts(*inports);
	new_inports->ports.length(ppp.size());
	unsigned int k = 0;
	foreach(const PortMap::value_type& port, ppp){
	  foreach(const BindingData::ProxyPort p, port.second) {
	    //begin nah change
	    assert(k < new_inports->ports.length());
	    //end nah change
	    new_inports->ports[k++] = p;
	    assert(string(p.proxyID) != "");
	    assert(string(p.ownerProxyID) != "");
	    assert(string(p.label) != "");
	  }
	}
	//cout << "NOW OVERWRITING INPORTS" << endl;
	overwriteWorkingMemory(inportsID, 
			       //BindingLocalOntology::PROXY_PORTS_TYPE,
			       new_inports,
			       cdl::BLOCKING);	
	//cout << "OVERWRITTEN INPORTS" << endl;
      } catch(const DoesNotExistOnWMException& _e) {
	log("could not update inports of " + proxyID + " : " + _e.what());
      }
    }
  }
  if(!proxies.empty()) {
    BindingData::BindTheseProxies* bindThese = new BindingData::BindTheseProxies();
    bindThese->proxyIDs.length(proxies.size());
    
    set<string>::const_iterator unbound_itr = proxies.begin();
    for(unsigned int i = 0 ; i < bindThese->proxyIDs.length() ; ++i,++unbound_itr) {
      bindThese->proxyIDs[i] = CORBA::string_dup(unbound_itr->c_str());
    }
    addToWorkingMemory(newDataID(), 
		       //BindingLocalOntology::BIND_THESE_PROXIES_TYPE, 
		       bindThese, 
		       cdl::BLOCKING);
  }
  cout << "Finished with removeInports " << endl;
}





} // namespace Binding 
