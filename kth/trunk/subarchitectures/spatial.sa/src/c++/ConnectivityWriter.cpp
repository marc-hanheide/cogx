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

    Marshalling::MarshallerPrx agg(getIceServer<Marshalling::Marshaller>("proxy.marshaller"));
    FeaturePtr feature = new Feature();
    feature->featlabel = "place_connectivity";
    feature->alternativeValues.push_back(new 
	binder::autogen::featvalues::IntegerValue(1,place2));
    agg->addFeature(type, uid, feature);
  }
  else {
    log("The property struct disappeared!");
  }
}

