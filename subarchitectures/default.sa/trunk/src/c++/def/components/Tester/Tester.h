/**
 * @author Andrzej Pronobis
 *
 * Declaration of the def::Tester class.
 */

#ifndef DEFAULT_TESTER_H
#define DEFAULT_TESTER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <DefaultData.hpp>

class QApplication;

namespace def
{

class TesterDialog;

/**
 * @author Andrzej Pronobis
 *
 * Tester for the Default.SA
 */
class Tester: public cast::ManagedComponent
{

public:

	/** Constructor. */
	Tester():_qApp(0), _dialog(0) {}

	/** Destructor. */
	virtual ~Tester() {}

	/** Sends a new query. */
	DefaultData::QdlQueryResults sendHFCQuery(std::string query);


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

	/** Id of the forward chainer server component.  */
	std::string _hfcServerName;

	/** Id of the QueryHandler component.  */
	std::string _queryHandlerName;

	/** ICE proxy to the forward chainer server interface. */
	DefaultData::HFCInterfacePrx _hfcInterfacePrx;

	/** ICE proxy to the QueryHandlerInterface. */
	DefaultData::QueryHandlerServerInterfacePrx _queryHandlerServerInterfacePrx;

	/** Qt application. */
	QApplication *_qApp;

	/** QT GUI dialog. */
	TesterDialog *_dialog;

}; // class Tester
} // namespace def

#endif // DEFAULT_TESTER_H



