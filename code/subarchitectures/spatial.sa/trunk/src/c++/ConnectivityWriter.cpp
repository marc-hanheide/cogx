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
    int place1 = prop->place1Id;
    int place2 = prop->place2Id;

    string type = "place";
    stringstream ss;
    ss << place1;
    string uid = ss.str();
    stringstream ss2;
    ss2 << place2;
    string uid2 = ss2.str();

    FeaturePtr feature = new Feature();
    feature->featlabel = "place_connectivity";
    feature->alternativeValues.push_back(new 
					 binder::autogen::featvalues::StringValue(1,getCASTTime(),uid2));
    m_marshaller->addFeature(type, uid, feature);
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

    if (wmc.type == "ADD") {
    FeaturePtr feature = new Feature();
    feature->featlabel = "gateway";
    feature->alternativeValues.push_back(new 
					 binder::autogen::featvalues::StringValue(1,getCASTTime(),"gateway"));
    m_marshaller->addFeature(type, uid, feature);
    }
    else if (wmc.type == "DELETE") {
      m_marshaller->deleteFeature(type, uid, "gateway");
    }
  }
}


