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
  sleepComponent(5000);
  println("faking stuff up");

  //copied from fake visual thingy
    
  WorkingMemoryPointerPtr origin = createWorkingMemoryPointer(getSubarchitectureID(), "blablabla", "anyDatatype");
  
  FeatureValuePtr relation = createStringValue ("relation", 0.8f);
  FeaturePtr feat1 = createFeatureWithUniqueFeatureValue ("type", relation);
  
  ProxyPtr proxy = createNewProxy (origin, 0.75f);
  
  addFeatureToProxy (proxy, feat1);
  
  addProxyToWM(proxy);

}
