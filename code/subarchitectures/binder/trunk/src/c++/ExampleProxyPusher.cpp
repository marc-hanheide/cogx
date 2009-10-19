#include "ExampleProxyPusher.hpp"


extern "C" {
  cast::interfaces::CASTComponentPtr
  newComponent() {
    return new ExampleProxyPusher();
  }
}



using namespace binder::autogen::core;
using namespace binder::autogen::featvalues;
using namespace cast;
using namespace cast::cdl;


void ExampleProxyPusher::start() {
  //must call super start to ensure that the reader sets up change
  //filters
  BindingWorkingMemoryReader::start();
}

void ExampleProxyPusher::runComponent() {
  sleepComponent(1000);
  println("faking stuff up");

  //copied from fake visual thingy

  cast::cdl::WorkingMemoryPointerPtr origin1 = new cast::cdl::WorkingMemoryPointer();
  origin1->address.subarchitecture = "no";
  origin1->address.id = "local";
  origin1->type = "data"; //uh oh, do we always need to include this?

  ProxyPtr proxy1 = createNewProxy(origin1, 1.0f);

  FeaturePtr feature = new Feature();
  feature->featlabel = "category";
  feature->alternativeValues.push_back(new
				   binder::autogen::featvalues::StringValue(1, getCASTTime(), "robot"));
  addFeatureToProxy(proxy1, feature);

  addProxyToWM(proxy1);

  sleepComponent(1000);

  WorkingMemoryPointerPtr origin2 = createWorkingMemoryPointer ("spatial.sa", "bong", "place");
  ProxyPtr proxy2 = createNewProxy (origin2, 1.0f);

  FeatureValuePtr zero = createIntegerValue (0 , 1.0f);
  FeaturePtr feat1b = createFeatureWithUniqueFeatureValue("place.id", zero);
  addFeatureToProxy (proxy2, feat1b);

  FeatureValuePtr explored = createBooleanValue (true , 1.0f);
  FeaturePtr feat2b = createFeatureWithUniqueFeatureValue("explored", explored);
  addFeatureToProxy (proxy2, feat2b);

  addProxyToWM(proxy2);

}
