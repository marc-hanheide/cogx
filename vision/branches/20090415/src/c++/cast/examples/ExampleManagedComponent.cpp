#include "ExampleManagedComponent.hpp"

#include <ChangeFilterFactory.hpp>




using namespace std;
using namespace cast;
using namespace cast::cdl;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new ExampleManagedComponent();
  }
}




void 
ExampleManagedComponent::start() {

  println("adding");
  addChangeFilter(createLocalTypeFilter<TestStructString>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ExampleManagedComponent>(this,
									    &ExampleManagedComponent::tellTheWorld));  
  println("added");

}

void
ExampleManagedComponent::tellTheWorld(const cast::cdl::WorkingMemoryChange & _wmc) {
  ostringstream outStream;
  outStream<<_wmc;
  println(outStream.str());
  println(getMemoryEntry<TestStructString>(_wmc.address)->dummy);
}



void
ExampleManagedComponent::runComponent() {
  Ice::CommunicatorPtr ic = getCommunicator();

  while (isRunning()) {
    try {
      println("running");
      static int slp = 1;
      sleep(slp++);    
      TestStructStringPtr tss = new TestStructString(getComponentID());
      addToWorkingMemory(newDataID(), tss);    
      // cdl::CASTTime ct = getCASTTime();
      // cout<<ct<<endl;    
    }
    catch (const Ice::Exception & e) {
      cout<<e.what()<<endl;
    }
  }

}
