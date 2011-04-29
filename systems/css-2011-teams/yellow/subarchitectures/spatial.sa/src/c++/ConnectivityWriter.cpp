#include "ConnectivityWriter.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <Ice/Ice.h>
#include <BinderEssentials.hpp>
#include <sstream>

using namespace std;
using namespace cast;
using namespace binder::autogen::core;
using namespace spatial;

extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new ConnectivityWriter();
  }
}

ConnectivityWriter::ConnectivityWriter()
{
}

ConnectivityWriter::~ConnectivityWriter()
{
}

void
ConnectivityWriter::start()
{
  addChangeFilter(createLocalTypeFilter<SpatialProperties::ConnectivityPathProperty>
      (cdl::ADD),
		  new MemberFunctionChangeReceiver<ConnectivityWriter>(this,
					&ConnectivityWriter::newConnectivity));
  addChangeFilter(createLocalTypeFilter<SpatialProperties::GatewayPlaceProperty>
      (cdl::WILDCARD),
		  new MemberFunctionChangeReceiver<ConnectivityWriter>(this,
					&ConnectivityWriter::changedGateway));
  m_marshaller = Marshalling::MarshallerPrx(getIceServer<Marshalling::Marshaller>("proxy.marshaller"));
}

void
ConnectivityWriter::stop()
{
}

void
ConnectivityWriter::runComponent()
{
}

void
ConnectivityWriter::configure(const std::map<std::string, std::string>& _config)
{
}

void
ConnectivityWriter::newConnectivity(const cdl::WorkingMemoryChange &wmc)
{
  SpatialProperties::ConnectivityPathPropertyPtr prop =
    getMemoryEntry<SpatialProperties::ConnectivityPathProperty>(wmc.address);
  if (prop != 0) {
    cast::cdl::WorkingMemoryPointerPtr origin = new cast::cdl::WorkingMemoryPointer();
    origin->address.subarchitecture = "spatial.sa";
    origin->address.id = "local";
    origin->type = "data";

    int place1 = prop->place1Id;
    int place2 = prop->place2Id;

    string type = "place";
    stringstream ss;
    ss << place1;
    string uid = ss.str();

    stringstream ss2;
    ss2 << place2;
    string uid2 = ss2.str();

    stringstream ss3;
    ss3 << place1 << "-" << place2;
    string relationUID = ss3.str();

    log("Adding connectivity relation between Place %s and %s", uid.c_str(), uid2.c_str());
    while(!m_marshaller->addRelation("connectivity", relationUID,
				     "place", uid,
				     "place", uid2,
				     1.0, origin)) {
      log("looping for failed relation addition");
    }

    // Add the "relationType" label feature
    FeaturePtr feature = new Feature();
    feature->featlabel = "connected";
    feature->alternativeValues.push_back(new
					 binder::autogen::featvalues::BooleanValue(1,getCASTTime(),true));
    m_marshaller->addFeature("connectivity", relationUID, feature);

    m_marshaller->commitFeatures("connectivity", relationUID);
  }
  else {
    log("The property struct disappeared!");
  }
}

void
ConnectivityWriter::changedGateway(const cdl::WorkingMemoryChange &wmc)
{
  SpatialProperties::GatewayPlacePropertyPtr prop =
    getMemoryEntry<SpatialProperties::GatewayPlaceProperty>(wmc.address);
  if (prop != 0) {
    int place = prop->placeId;

    string type = "place";
    stringstream ss;
    ss << place;
    string uid = ss.str();

//     if (wmc.type == "ADD") {
//       FeaturePtr feature = new Feature();
//       feature->featlabel = "gateway";
//       feature->alternativeValues.push_back(new
// 	  binder::autogen::featvalues::StringValue(1,getCASTTime(),"gateway"));
//       m_marshaller->addFeature(type, uid, feature);
//       m_marshaller->commitFeatures(type, uid);
//     }
//     else if (wmc.type == "DELETE") {
//       m_marshaller->deleteFeature(type, uid, "gateway");
//       m_marshaller->commitFeatures(type, uid);
//     }

    if (wmc.operation == cdl::ADD) {
      FeaturePtr feature = new Feature();
      feature->featlabel = "gateway";
      feature->alternativeValues.push_back(new
	  binder::autogen::featvalues::StringValue(1,getCASTTime(),"gateway"));
      m_marshaller->addFeature(type, uid, feature);
      m_marshaller->commitFeatures(type, uid);
    }
    else if (wmc.operation == cdl::DELETE) {
      m_marshaller->deleteFeature(type, uid, "gateway");
      m_marshaller->commitFeatures(type, uid);
    }
  }
}


