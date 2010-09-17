/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ComaRoomUpdater class.
 */

#ifndef CONCEPTUAL_COMAROOMUPDATER_H
#define CONCEPTUAL_COMAROOMUPDATER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

class QApplication;

namespace conceptual
{


/**
 * @author Andrzej Pronobis
 *
 * Main query handler for the Conceptual.SA
 */
class ComaRoomUpdater: public cast::ManagedComponent
{

public:

	/** Constructor. */
	ComaRoomUpdater() {}

	/** Destructor. */
	virtual ~ComaRoomUpdater() {}


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


}; // class ComaRoomUpdater
} // namespace def

#endif // CONCEPTUAL_COMAROOMUPDATER_H



