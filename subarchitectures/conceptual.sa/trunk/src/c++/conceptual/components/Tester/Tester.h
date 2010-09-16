/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::Tester class.
 */

#ifndef CONCEPTUAL_TESTER_H
#define CONCEPTUAL_TESTER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

class QApplication;

namespace conceptual
{

class TesterDialog;

/**
 * @author Andrzej Pronobis
 *
 * Tester for the Conceptual.SA
 */
class Tester: public cast::ManagedComponent
{


public:

	/** Constructor. */
	Tester():_qApp(0), _dialog(0) {}

	/** Destructor. */
	virtual ~Tester() {}

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
	ConceptualData::QueryHandlerServerInterfacePrx _queryHandlerServerInterfacePrx;

	/** Qt application. */
	QApplication *_qApp;

	/** QT GUI dialog. */
	TesterDialog *_dialog;

}; // class Tester
} // namespace def

#endif // CONCEPTUAL_TESTER_H



