/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::Observer class.
 */

#ifndef CONCEPTUAL_OBSERVER_H
#define CONCEPTUAL_OBSERVER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

namespace conceptual
{

/**
 * @author Andrzej Pronobis
 *
 * Component observing changes on other working memories
 * which correspond to changes in spatial knowledge represented
 * by other layers of the representation.
 */
class Observer: public cast::ManagedComponent
{

public:

	/** Constructor. */
	Observer()
	{}

	/** Destructor. */
	virtual ~Observer()
	{}


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

	/** Initializes the world state. */
	void initializeWorldState();

	/** Adds the worldstate to the working memory. */
	void addInitialWorldState();

	/** Updates the world state on the WM. */
	void updateWorldState();

	/** Executed when coma room changes. */
	void comaRoomChanged(const cast::cdl::WorkingMemoryChange &wmChange);


private:

	/** Current state of the world as much as
	 * the conceptual.sa is concerned. */
	ConceptualData::WorldStatePtr _worldStatePtr;

	/** Working memory ID of the world state struct. */
	std::string _worldStateId;

}; // class Observer
} // namespace

#endif // CONCEPTUAL_OBSERVER_H



