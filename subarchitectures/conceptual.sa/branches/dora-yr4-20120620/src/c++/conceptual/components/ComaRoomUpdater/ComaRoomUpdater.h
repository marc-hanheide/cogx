/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ComaRoomUpdater class.
 */

#ifndef CONCEPTUAL_COMAROOMUPDATER_H
#define CONCEPTUAL_COMAROOMUPDATER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

namespace conceptual
{

/**
 * @author Andrzej Pronobis
 *
 * Updates the coma room structs on the coma.sa WM with
 * information about room categories.
 */
class ComaRoomUpdater: public cast::ManagedComponent
{

public:

	/** Constructor. */
	ComaRoomUpdater();

	/** Destructor. */
	virtual ~ComaRoomUpdater();


protected:

	/** Called by the framework to configure the component. */
	virtual void configure(const std::map<std::string,std::string> & _config);

	/** Called by the framework after configuration, before run loop. */
	virtual void start();

	/** The main run loop. */
	virtual void runComponent();

	/** Called by the framework after the run loop finishes. */
	virtual void stop();


private:

	/** World state changed, infer and then update the coma room structs. */
	void worldStateChanged(const cast::cdl::WorkingMemoryChange &wmChange);


private:

	  pthread_cond_t _worldStateChangedSignalCond;
	  pthread_mutex_t _worldStateChangedSignalMutex;

	  /** True if the world state has changed since the last time we checked. */
	  bool _worldStateChanged;

	  /** Map ComaRoom Id -> WMAddress. */
	  std::map<int, cast::cdl::WorkingMemoryAddress> _comaRoomIdToWmAddressMap;

	  /** Name of the QueryHandler component.  */
	  std::string _queryHandlerName;

	  /** ICE proxy to the QueryHandlerInterface. */
	  ConceptualData::QueryHandlerServerInterfacePrx _queryHandlerServerInterfacePrx;

}; // class ComaRoomUpdater
} // namespace

#endif // CONCEPTUAL_COMAROOMUPDATER_H



