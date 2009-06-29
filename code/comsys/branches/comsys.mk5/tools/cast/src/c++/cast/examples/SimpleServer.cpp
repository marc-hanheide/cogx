#include "SimpleServer.hpp"

using namespace std;
using namespace Ice;
using namespace cast;
using namespace cast::cdl;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new SimpleServer();
  }
}



void
SimpleServer::start() {
  registerIceServer<cast::CASTComponent,WordServerAsComponent>(getComponentPointer());
}
