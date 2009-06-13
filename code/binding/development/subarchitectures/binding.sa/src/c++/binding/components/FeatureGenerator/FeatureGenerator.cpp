#include "FeatureGenerator.hpp"
#include <binding/idl/BindingQueries.hh>


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
    return new Binding::FeatureGenerator(_id);
  }
}

namespace Binding {

  using namespace BindingData;
  using namespace BindingQueries;
  using namespace BindingFeatures;
  
  FeatureGenerator::FeatureGenerator(const string &_id) : 
    
    WorkingMemoryAttachedComponent(_id),
    AbstractBinder(_id),
    handler(*this) { 
  }
  
  void
  FeatureGenerator::start() {
    
    AbstractBinder::start();
    
    addChangeFilter(createLocalTypeFilter<BindingQueries::FeatureRequest>(cdl::ADD),
		    new MemberFunctionChangeReceiver<FeatureGenerator>(this,
								       &FeatureGenerator::newFeatureRequest));

    addChangeFilter(createLocalTypeFilter<BindingQueries::FeatureGenerationRegistration>(cdl::ADD),
		    new MemberFunctionChangeReceiver<FeatureGenerator>(this,
								       &FeatureGenerator::registerFeature));

    addChangeFilter(createLocalTypeFilter<BindingQueries::FeatureGenerationCommand>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<FeatureGenerator>(this,
								       &FeatureGenerator::generationComplete));
    
  }
  


  FeatureGenerator::~FeatureGenerator() {
    
  }


  void
  FeatureGenerator::newFeatureRequest(const cdl::WorkingMemoryChange & _wmc) {
    shared_ptr<const CASTData<FeatureRequest> > request(getWorkingMemoryEntry<FeatureRequest>(_wmc.address));
    string type(request->getData()->type);
    string reqSA(request->getData()->subarchitecture);
    log("feature %s requested by subarchitecture %s",type.c_str(),reqSA.c_str());
    
    
    //get the list of sas that can provide this feature
    StringSetMap::const_iterator subarchsIt(feature2sa.find(type));
    
    //if we can't find this type
    if(subarchsIt == feature2sa.end()) {
      log("no subarchitecture provides feature: %s", type.c_str());
      answerRequest(_wmc.address,request,BindingData::FALSETB);
    }
    else {

      
      
      //load the relevant binding structures
      //ProxyPtr& reqProxy(handler.loadProxy(string(request->getData()->proxyID)));
      //it must be bound
      //assert(reqProxy->bound());
      //load the union with the proxy in it
      //UnionPtr& reqUnion(handler.loadUnion(reqProxy->bindingUnionID()));
      //get union features
      //const FeatureSetWithRepetitions& fs(reqUnion->featureSetWithRepetitions());
      //particularly the source id ones
      //fs.
      //const std::set<std::string>& proxies(reqUnion->) const;


      //load the proxy indicated in the request
      //long winded, but easier to reuse code
      set<string> reqIDs;
      reqIDs.insert(string(request->getData()->proxyID));
      ProxySet reqProxySet(handler.loadProxies(reqIDs)); 
      assert(reqProxySet.size() == 1);

      //now load all the proxies bound to this one
      ProxySet boundProxies(handler.extractProxiesFromProxies(reqProxySet,
								BoundProxyFromProxyExtractor()));

      log("%d bound proxies", boundProxies.size());



      const StringSet& subarchs(subarchsIt->second);
      for(StringSet::const_iterator i = subarchs.begin();
	  i != subarchs.end(); ++i) {
	const string & providingSA(*i);

	log("providing sa: %s", providingSA.c_str());


	if(providingSA == reqSA) {
	  log("not querying requesting SA");
	}
	else {
	 
	  //TODO inefficient, with duplicate checks, but lets just get
	  //it working

	  //must have a proxy from there
	  //look at each bound proxy to find a proxy from that sa	  
	  ProxyPtr genProxy; //starts null
	  for(ProxySet::const_iterator i = boundProxies.begin();
	      i != boundProxies.end(); ++i) {	    
	    //load Source id for this proxy
	    const SourceID & sid(i->second->getFeature<SourceID>());
	    string sidSA(sid.sourceID);
	    log("proxy %s has source id %s",i->first.c_str(), sidSA.c_str());
	    if(sidSA == providingSA) {
	      log("found a proxy for the sa");
	      genProxy = i->second;
	      break;
	    }	    
	  }
	  
	  if(genProxy) {
	    log("providing sa has proxy");
	  }
	  else {
	    log("providing sa does not have proxy");
	  }

	  //if that sa has already provided a proxy with that feature,
	  //then ignore it too
	  
	  

	  //

	}

      }
    }
		    
  }
  
    

  void 
  FeatureGenerator::answerRequest(const cast::cdl::WorkingMemoryAddress & _wma,
				  boost::shared_ptr<const cast::CASTData<BindingQueries::FeatureRequest> > _request,
				  const BindingData::TriBool & _successful)  {
    //copy the req struct
    FeatureRequest * req = new FeatureRequest(*(_request->getData()));
  req->processed = true;
  req->successful = _successful;
  overwriteWorkingMemory(_wma,req);
//   ostringstream os;
//   os<<"request answered: "<<_wma<<" "<<_successful<<endl;
//   log(os.str());
}
  

void 
  FeatureGenerator::registerFeature(const cdl::WorkingMemoryChange & _wmc) {

    shared_ptr<const CASTData<FeatureGenerationRegistration> > registration(getWorkingMemoryEntry<FeatureGenerationRegistration>(_wmc.address));
    string type(registration->getData()->type);
    string sa(registration->getData()->subarchitecture);
    feature2sa[type].insert(sa);
    log("registering feature %s from subarchitecture %s",type.c_str(),sa.c_str());
    
    

  }

  void
  FeatureGenerator::generationComplete(const cdl::WorkingMemoryChange & _wmc) {

  }

} // namespace Binding 

