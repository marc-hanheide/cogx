/**
 * @author Andrzej Pronobis
 *
 * Declaration of the ConceptualQueryHandler class.
 */

#ifndef CONCEPTUALQUERYHANDLER_H
#define CONCEPTUALQUERYHANDLER_H

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
class ConceptualQueryHandler: public cast::ManagedComponent
{

public:

	/** Constructor. */
	ConceptualQueryHandler() {}

	/** Destructor. */
	virtual ~ConceptualQueryHandler() {}

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


}; // class ConceptualQueryHandler
} // namespace def

#endif // CONCEPTUALQUERYHANDLER_H



