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

  WorkingMemoryPointerPtr origin1 = createWorkingMemoryPointer("spatial.sa", "bing", "robot");

  ProxyPtr proxy = createNewProxy (origin1, 1.0f);

  FeatureValuePtr robot = createStringValue ("robot", 1.0f);
  FeaturePtr feat1 = createFeatureWithUniqueFeatureValue("category", robot);
  addFeatureToProxy (proxy, feat1);

  addProxyToWM(proxy);

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
