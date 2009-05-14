#include "cast/core/CASTCompositeOntology.hpp"
#include <binding/ontology/BindingOntologyFactory.hpp>
#include <manipulation/ManipulationOntologyFactory.hpp>
#include "ManipulationBindingMonitor.h"

namespace Binding
{

ManipulationBindingMonitor::ManipulationBindingMonitor(const string _id)
  : WorkingMemoryAttachedComponent(_id), AbstractMonitor(_id)
{
  CASTCompositeOntology *pComposite = new CASTCompositeOntology();
  pComposite->addOntology(ManipulationOntologyFactory::getOntology());
  pComposite->addOntology(BindingOntologyFactory::getOntology());
  setOntology(pComposite);

  setReceiveXarchChangeNotifications(true);
}

void ManipulationBindingMonitor::start()
{
  AbstractMonitor::start();

  addChangeFilter(createLocalTypeFilter<Location>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ManipulationBindingMonitor>(this,
									       &ManipulationBindingMonitor::locationAdded));
  
  addChangeFilter(createLocalTypeFilter<Location>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<ManipulationBindingMonitor>(this,
									       &ManipulationBindingMonitor::locationUpdated));
  
  addChangeFilter(createLocalTypeFilter<Location>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<ManipulationBindingMonitor>(this,
									       &ManipulationBindingMonitor::locationDeleted));
  
  log("started");  // HACK
  printf(">>>>>>>>>>>>>>>> started\n");  // HACK
}

void ManipulationBindingMonitor::locationAdded(const cdl::WorkingMemoryChange& _wmc)
{
  /*
  // get the location feature from the binding WM
  shared_ptr<const CASTData<BindingData::Feature::Location> > loc_ptr = 
    getWorkingMemoryEntry<BindingData::Feature::Location>(_wmc.m_address);
  shared_ptr<const BindingData::Feature::Location> loc = loc_ptr->getData();

  // add our two cents: start a new proy
  startNewBasicProxy();

  // add reachability feature
  BindingData::Feature::Reachable reachable;
  // HACK: find out whether the object is actually reachable
  reachable.m_reachable = loc->m_location.m_x < 0.5 && loc->m_location.m_y < 0.5;
  addFeatureToCurrentProxy(reachable);

  // add existing prixy ID feature, to explicitely bind our new proxy to that
  // proxy of which the above location is a feature of
  BindingData::Feature::ExistingProxyID ex_prox;
  ex_prox.m_existingProxyID = loc->m_immediateProxyID;

  // finish the new proxy
  string proxyAddr = storeCurrentProxy();
  m_sourceProxyMapping[loc_ptr->getID()] = proxyAddr;

  // an start off binding of the new proxy
  bindNewProxies();
*/
  // HACK
  log("added location");//proxy '%s'", proxyAddr.c_str());
  printf(">>>>>>>>>>> added location\n");//proxy '%s'", proxyAddr.c_str());
}

void ManipulationBindingMonitor::locationUpdated(const cdl::WorkingMemoryChange& _wmc)
{
  // HACK
  log("updated location");
  printf("i>>>>>>>>>>>>>>>> updated location\n");
}

void ManipulationBindingMonitor::locationDeleted(const cdl::WorkingMemoryChange& _wmc)
{
  // HACK
  log("delted location");
  printf(">>>>>>>>>>>>>>>>>. delted location\n");
}

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
FrameworkProcess* newComponent(const string &_id) {
  return new Binding::ManipulationBindingMonitor(_id);
}
}

}

