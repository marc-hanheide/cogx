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


void ExampleProxyPusher::start() {
  //must call super start to ensure that the reader sets up change
  //filters
  BindingWorkingMemoryReader::start();
}

void ExampleProxyPusher::runComponent() {
  sleepComponent(5000);
  println("faking stuff up");

  //copied from fake visual thingy
  ProxyPtr proxy = createNewProxy (getSubarchitectureID(), 0.75f);
  
  FeatureValuePtr mug = createStringValue ("mug", 0.8f);
  FeaturePtr feat1 = createFeatureWithUniqueFeatureValue ("obj_label", mug);
  addFeatureToProxy (proxy, feat1);
  
  FeatureValuePtr blue = createStringValue ("blue", 0.95f);
  FeaturePtr feat2 = createFeatureWithUniqueFeatureValue ("colour", blue);
  addFeatureToProxy (proxy, feat2);
  
  addProxyToWM(proxy);

}
