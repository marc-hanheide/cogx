/**
 * @author Andrzej Pronobis
 *
 * Declaration of the ConceptualObserver class.
 */

#ifndef CONCEPTUALOBSERVER_H
#define CONCEPTUALOBSERVER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

class QApplication;

namespace def
{


/**
 * @author Andrzej Pronobis
 *
 * Main query handler for the Conceptual.SA
 */
class ConceptualObserver: public cast::ManagedComponent
{

public:

	/** Constructor. */
	ConceptualObserver() {}

	/** Destructor. */
	virtual ~ConceptualObserver() {}

	/** Sends a new query. */
	DefaultData::DiscreteProbabilityDistribution sendQuery(std::string query);


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


}; // class ConceptualObserver
} // namespace def

#endif // CONCEPTUALOBSERVER_H



