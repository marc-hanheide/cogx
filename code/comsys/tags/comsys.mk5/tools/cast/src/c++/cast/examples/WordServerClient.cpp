#include "WordServerClient.hpp"

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
    return new WordServerClient();
  }
}



void
WordServerClient::runComponent() {

  WordServerPrx agg(getIceServer<WordServer>("aggregate.server"));
  println(agg->getNewWord());

  WordServerAsComponentPrx impl(getIceServer<WordServerAsComponent>("implements.server"));
  println(impl->getNewWord());
}
