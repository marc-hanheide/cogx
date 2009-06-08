#ifdef COMSYS_INCLUDED
//#include <comsys/ontology/ComsysOntology.hpp>
#endif
#include "binding/utils/BindingUtils.hpp"
#include "binding/BindingException.hpp"
#include "BindingWorkingMemory.hpp"

using namespace std;
using namespace boost;
using namespace cast;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::BindingWorkingMemory(_id);
  }
}

namespace Binding {

  BindingWorkingMemory::BindingWorkingMemory(const string &_id)
    : 
      SubarchitectureWorkingMemory(_id)
 {
/*#ifdef COMSYS_INCLUDED
   static CASTCompositeOntology ont;
   ont.addOntology(&BindingOntologyFactory::getOntology());
   ont.addOntology(&comsys::ComsysOntology::construct());
   setOntology(&ont);
#else
   setOntology(&BindingOntologyFactory::getOntology());
#endif
*/
   // binding wm should broadcast 
   setSendXarchChangeNotifications(true);
   
  }

  BindingWorkingMemory::~BindingWorkingMemory()
  {
  }


} // namespace Binding
