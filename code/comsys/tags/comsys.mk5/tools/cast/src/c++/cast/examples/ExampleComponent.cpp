#include "ExampleComponent.hpp"

using namespace std;
using namespace cast;
using namespace cast::cdl;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new ExampleComponent();
  }
}




void
ExampleComponent::runComponent() {

  //CASTTime justasec(castTimeMicros(1000000));
  CASTTime justasec(castTimeMillis(1000));
  //CASTTime justasec(castTimeSeconds(1));

  for (unsigned int i = 0; i < 50 && isRunning(); ++i) {
    if(isRunning()) {
      println("running, but not really doing anything");
      
      CASTTime now(getCASTTime());

      cout<<(now)<<endl;
      cout<<(justasec)<<endl;
      cout<<(now - justasec)<<endl;
      cout<<(justasec - now)<<endl;


      sleepComponent(1000);
    }
  }
}
