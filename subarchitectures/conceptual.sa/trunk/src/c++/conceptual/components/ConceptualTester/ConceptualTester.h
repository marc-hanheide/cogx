/**
 * @author Andrzej Pronobis
 *
 * Declaration of the ConceptualTester class.
 */

#ifndef CONCEPTUALTESTER_H
#define CONCEPTUALTESTER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

class QApplication;

namespace def
{

class ConceptualTesterDialog;

/**
 * @author Andrzej Pronobis
 *
 * Tester for the Conceptual.SA
 */
class ConceptualTester: public cast::ManagedComponent
{


public:

	/** Constructor. */
	ConceptualTester():_qApp(0), _dialog(0) {}

	/** Destructor. */
	virtual ~ConceptualTester() {}

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

	/** Id of the QueryHandler component.  */
	std::string _queryHandlerName;

	/** ICE proxy to the QueryHandlerInterface. */
	ConceptualData::QueryHandlerInterfacePrx _queryHandlerInterfacePrx;

	/** Qt application. */
	QApplication *_qApp;

	/** QT GUI dialog. */
	ConceptualTesterDialog *_dialog;

}; // class ConceptualTester
} // namespace def

#endif // CONCEPTUALTESTER_H



