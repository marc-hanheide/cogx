#include "ProxyMarshaller.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace boost;
using namespace cast;
using namespace spatial;
using namespace Ice;
using namespace binder::autogen::core;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new ProxyMarshaller();
  }
}

ProxyMarshaller::ProxyMarshaller()
{
}


ProxyMarshaller::~ProxyMarshaller()
{
}

void
ProxyMarshaller::start()
{
  addChangeFilter(createLocalTypeFilter<SpatialData::Place>(cast::cdl::ADD),
		  new cast::MemberFunctionChangeReceiver<ProxyMarshaller>(this,
		    &ProxyMarshaller::newPlace));    

  addChangeFilter(createLocalTypeFilter<SpatialData::Place>(cast::cdl::OVERWRITE),
		  new cast::MemberFunctionChangeReceiver<ProxyMarshaller>(this,
		    &ProxyMarshaller::changedPlace));    

  addChangeFilter(createLocalTypeFilter<SpatialData::Place>(cast::cdl::DELETE),
		  new cast::MemberFunctionChangeReceiver<ProxyMarshaller>(this,
		    &ProxyMarshaller::deletedPlace));    
}

void
ProxyMarshaller::stop()
{
}

void
ProxyMarshaller::runComponent()
{
}

void
ProxyMarshaller::configure(const std::map<std::string, std::string>& _config)
{
  log("Configure entered");

  if (_config.find("-bsa") != _config.end()) {
    m_bindingSA=_config.find("-bsa")->second;
  } else if (_config.find("--bsa") != _config.end()) {
    m_bindingSA=_config.find("--bsa")->second;
  } else {
    m_bindingSA="binding.sa";
  }

  Marshalling::MarshallerPtr servant = new MarshallingServer(this);
  registerIceServer<Marshalling::Marshaller, Marshalling::Marshaller>(servant);
}

void
ProxyMarshaller::MarshallingServer::addProxy(const string & type, const string & UID,
					     double probExists, 
					     const cast::cdl::WorkingMemoryPointerPtr & origin, 
					     const Ice::Current &_context)
{
  m_pOwner->addProxy(type, UID, probExists, origin);
}

void
ProxyMarshaller::addProxy(const string & type, const string & UID,
			  double probExists,
			  const cast::cdl::WorkingMemoryPointerPtr & origin)
{
  map<string, InternalProxy> &typeMap =
    m_proxyTypeMap[type];

  map<string, InternalProxy>::iterator it = 
    typeMap.find(UID);

  if (it == typeMap.end()) {
    // create new proxy
    log("creating a new proxy.");

    typeMap[UID].proxy = createNewProxy(origin, probExists);
    //typeMap[UID].proxy->entityID = newDataID();
    //typeMap[UID].proxy->subarchId = getSubarchitectureID();
    //typeMap[UID].proxy->probExists = probExists;
    //typeMap[UID].proxy->distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(_newPlaceProxy);
    typeMap[UID].onBinder = false; //Not represented on binder yet

    //updateInternalProxy(typeMap[UID]); //Don't upload it until some features are added
  }
  else {
    // The proxy already existed.
  }
}

void
ProxyMarshaller::MarshallingServer::deleteProxy(const string & type, const string & UID,
    const Ice::Current &_context)
{
  m_pOwner->deleteProxy(type, UID);
}

void
ProxyMarshaller::deleteProxy(const string & type, const string & UID)
{
  if (m_proxyTypeMap.find(type) != m_proxyTypeMap.end()) {
    map<string, InternalProxy> &typeMap =
      m_proxyTypeMap[type];
    if (typeMap.find(UID) != typeMap.end()) {
      InternalProxy &intProxy = typeMap[UID];
      if (intProxy.onBinder) {
	// Delete the proxy from the Binder
	deleteEntityInWM(intProxy.proxy);
      }
      typeMap.erase(typeMap.find(UID));
    }
    else {
      log("addFeature: Proxy type %s, UID %s not found!", type.c_str(),
	  UID.c_str());
    }
  }
  else {
    log("addFeature: Proxy type %s not found!", type.c_str(),
	UID.c_str());
  }
}

void
ProxyMarshaller::MarshallingServer::addFeature(const string & proxyType, 
    const string & proxyUID, const binder::autogen::core::FeaturePtr &feature,
	const Ice::Current &_context)
{
  m_pOwner->addFeature(proxyType, proxyUID, feature);
}

void
ProxyMarshaller::addFeature(const string & proxyType, const string & proxyUID, 
	const binder::autogen::core::FeaturePtr feature)
{
  if (m_proxyTypeMap.find(proxyType) != m_proxyTypeMap.end()) {
    map<string, InternalProxy> &typeMap =
      m_proxyTypeMap[proxyType];
    if (typeMap.find(proxyUID) != typeMap.end()) {
      InternalProxy &intProxy = typeMap[proxyUID];
      vector<FeaturePtr>::iterator it = intProxy.proxy->features.begin();
//      for (; it != intProxy.proxy->features.end();
//	  it++) {
//	if ((*it)->featlabel == feature->featlabel) {
//	  break;
//	}
//      }
//      if (it != intProxy.proxy->features.end()) {
//	log("addFeature: Feature %s already exists in proxy type %s, UID %s!", 
//	    feature->featlabel.c_str(), proxyType.c_str(), proxyUID.c_str());
//      }
//      else {
	// Add the feature
	intProxy.proxy->features.push_back(feature);

	updateInternalProxy(typeMap[proxyUID]);
//      }
    }
    else {
      log("addFeature: Proxy type %s, UID %s not found!", proxyType.c_str(),
	  proxyUID.c_str());
    }
  }
  else {
    log("addFeature: Proxy type %s not found!", proxyType.c_str(),
	proxyUID.c_str());
  }
}

void
ProxyMarshaller::MarshallingServer::deleteFeature(const string & proxyType, const string & proxyUID,
	const string & featlabel,
	const Ice::Current &_context)
{
  m_pOwner->deleteFeature(proxyType, proxyUID, featlabel);
}

void
ProxyMarshaller::deleteFeature(const string & proxyType, const string & proxyUID,
	const string & featlabel)
{
  if (m_proxyTypeMap.find(proxyType) != m_proxyTypeMap.end()) {
    map<string, InternalProxy> &typeMap =
      m_proxyTypeMap[proxyType];
    if (typeMap.find(proxyUID) != typeMap.end()) {
      InternalProxy &intProxy = typeMap[proxyUID];
      vector<FeaturePtr>::iterator it = intProxy.proxy->features.begin();
      for (; it != intProxy.proxy->features.end();) {
	if ((*it)->featlabel == featlabel) {
	  it = intProxy.proxy->features.erase(it);
	  log("Erasing feature %s; %i remaining", featlabel.c_str(), intProxy.proxy->features.size());
	}
	else {
	  it++;
	}
      }
      if (it == intProxy.proxy->features.end()) {
	log("deleteFeature: Feature %s not found in proxy type %s, UID %s!", 
	    featlabel.c_str(), proxyType.c_str(), proxyUID.c_str());
      }
    }
    else {
      log("deleteFeature: Proxy type %s, UID %s not found!", proxyType.c_str(),
	  proxyUID.c_str());
    }
  }
  else {
    log("deleteFeature: Proxy type %s not found!", proxyType.c_str(),
	proxyUID.c_str());
  }
}

void
ProxyMarshaller::MarshallingServer::publishProxy(const string & proxyType, const string & proxyUID,
    const Ice::Current &_context)
{
  m_pOwner->publishProxy(proxyType, proxyUID);
}
  
void
ProxyMarshaller::publishProxy(const string & proxyType, const string & proxyUID)
{
  // All proxies are always published so this does nothing
}

void
ProxyMarshaller::updateInternalProxy(InternalProxy &intProxy)
{
  // Current function: Always maintain all proxies on the Binder

  // Check if proxy exists on Binder
  if (intProxy.onBinder) {
    // Proxy exists on Binder; overwrite its Features
    log("Proxy existed on Binder; overwrite its features");
    log("Proxy has %i features.", intProxy.proxy->features.size());
    overwriteProxyInWM(intProxy.proxy);
  }
  else {
    // Proxy is new; add it to Binder
    log("Proxy new on Binder; create it");
    log("Proxy has %i features.", intProxy.proxy->features.size());
    addProxyToWM(intProxy.proxy);
    intProxy.onBinder = true;
  }
}

void
ProxyMarshaller::newPlace(const cast::cdl::WorkingMemoryChange &_wmc)
{
  SpatialData::PlacePtr place = getMemoryEntry<SpatialData::Place>(_wmc.address);
  if (place != 0) {
    stringstream ss;
    ss << place->id;
    log("Adding proxy (place, %s)", ss.str().c_str());
    addProxy("place", ss.str(), 1.0, createWorkingMemoryPointer(_wmc.address, _wmc.type));
    m_PlaceAddressToIDMap[_wmc.address.id] = place->id;
    
    // Add place id feature
    log("Adding place_id feature");
    FeaturePtr feature = createFeature("place_id");
    feature->alternativeValues.push_back(createStringValue(ss.str(), 1));
	//binder::autogen::featvalues::IntegerValue(1, place->id));
    addFeature("place", ss.str(), feature);

    // Add Placeholder feature
    if (place->status == SpatialData::PLACEHOLDER) {
      log("Adding explored feature");
      feature = createFeature("explored");
      feature->alternativeValues.push_back(createStringValue("false", 1)); 
      //binder::autogen::featvalues::IntegerValue(1,1));
      addFeature("place", ss.str(), feature);
    }
    else {
      log("Adding non-explored feature");
      feature = createFeature("explored");
      feature->alternativeValues.push_back(createStringValue("true", 1)); 
      //binder::autogen::featvalues::IntegerValue(1,1));
      addFeature("place", ss.str(), feature);
    }
  }
}

void
ProxyMarshaller::changedPlace(const cast::cdl::WorkingMemoryChange &_wmc)
{
  SpatialData::PlacePtr place = getMemoryEntry<SpatialData::Place>(_wmc.address);
  if (place != 0) {
    stringstream ss;
    ss << place->id;
    log("Changed place changes proxy (place, %s)", ss.str().c_str());

    // Add Placeholder feature
    if (place->status == SpatialData::PLACEHOLDER) {
      deleteFeature("place", ss.str(), "explored");
      FeaturePtr feature = createFeature("explored");
      feature->alternativeValues.push_back(createStringValue("false",1));
          //binder::autogen::featvalues::IntegerValue(1,1));
      addFeature("place", ss.str(), feature);
    }
    else {
      deleteFeature("place", ss.str(), "explored");
      FeaturePtr feature = createFeature("explored");
      feature->alternativeValues.push_back(createStringValue("true",1));
          //binder::autogen::featvalues::IntegerValue(1,1));
      addFeature("place", ss.str(), feature);
    }
  }
}

void
ProxyMarshaller::deletedPlace(const cast::cdl::WorkingMemoryChange &_wmc)
{
  map<string, int>::iterator it = 
    m_PlaceAddressToIDMap.find(_wmc.address.id);

  if (it != m_PlaceAddressToIDMap.end()) {
    stringstream ss;
    ss << it->second;
    log("Deleting proxy for (place, %s)", ss.str().c_str());
    deleteProxy("place", ss.str());
  }
}
