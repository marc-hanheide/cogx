#include "SimpleAggregateServer.hpp"

using namespace std;
using namespace Ice;
using namespace cast;
using namespace cast::cdl;
using namespace cast::examples::autogen;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new SimpleAggregateServer();
  }
}



void
SimpleAggregateServer::start() {
  
  //register the server
  WordServerPtr servant = new WordServerI();
  registerIceServer<WordServer,WordServer>(servant);

}
