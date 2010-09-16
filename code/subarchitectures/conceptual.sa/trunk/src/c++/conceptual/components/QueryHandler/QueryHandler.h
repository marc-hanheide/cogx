/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::QueryQueryHandler class.
 */

#ifndef CONCEPTUAL_QUERYHANDLER_H
#define CONCEPTUAL_QUERYHANDLER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

class QApplication;

namespace conceptual
{


/**
 * @author Andrzej Pronobis
 *
 * Component that handles probabilistic queries
 * sent to the Conceptual.SA.
 */
class QueryHandler: public cast::ManagedComponent
{

	class Server: public ConceptualData::QueryHandlerServerInterface
	{
	public:

		/** Constructor. */
		Server(QueryHandler *queryHandler) : _queryHandler(queryHandler)
		{}

		virtual DefaultData::DiscreteProbabilityDistribution query(const std::string &queryStr, const Ice::Current &);

	private:

		/** Pointer to the owner of the server. */
		QueryHandler *_queryHandler;
    };


public:

	/** Constructor. */
	QueryHandler() {}

	/** Destructor. */
	virtual ~QueryHandler() {}


protected:
	/** Called by the framework to configure the component. */
	virtual void configure(const std::map<std::string,std::string> & _config);

	/** Called by the framework after configuration, before run loop. */
	virtual void start();

	/** The main run loop. */
	virtual void runComponent();

	/** Called by the framework after the run loop finishes. */
	virtual void stop();


}; // class ConceptualQueryHandler
} // namespace def

#endif // CONCEPTUAL_QUERYHANDLER_H



