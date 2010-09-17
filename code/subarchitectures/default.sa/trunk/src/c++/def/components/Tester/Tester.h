/**
 * @author Andrzej Pronobis
 *
 * Declaration of the def::Tester class.
 */

#ifndef DEFAULT_TESTER_H
#define DEFAULT_TESTER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <DefaultData.hpp>
#include <ComaData.hpp>

namespace def
{

/**
 * @author Andrzej Pronobis
 *
 * Tester for the Default.SA. It tests Default.SA
 * by using its external interfaces in a way external
 * components should use them.
 */
class Tester: public cast::ManagedComponent
{

public:

	/** Constructor. */
	Tester(): _hfcServerAvailable(false), _queryHandlerAvailable(false)
	{}

	/** Destructor. */
	virtual ~Tester() {}


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
	/** Sends a new query to the QueryHandler. */
	SpatialProbabilities::ProbabilityDistribution sendQueryHandlerQuery(const std::string &query) const;

	/** Sends a new query. */
	comadata::QueryResults sendHFCServerQuery(const std::string &query) const;


private:

	/** Id of the forward chainer server component.  */
	std::string _hfcServerName;

	/** Set to true if the QueryHandler is available. */
	bool _hfcServerAvailable;

	/** Id of the QueryHandler component.  */
	std::string _queryHandlerName;

	/** Set to true if the QueryHandler is available. */
	bool _queryHandlerAvailable;

	/** ICE proxy to the forward chainer server interface. */
	comadata::HFCInterfacePrx _hfcInterfacePrx;

	/** ICE proxy to the QueryHandlerInterface. */
	DefaultData::QueryHandlerServerInterfacePrx _queryHandlerServerInterfacePrx;


}; // class Tester
} // namespace def

#endif // DEFAULT_TESTER_H



