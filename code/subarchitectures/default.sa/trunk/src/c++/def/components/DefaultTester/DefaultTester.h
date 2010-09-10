/**
 * @author Andrzej Pronobis
 *
 * Declaration of the DefaultTester class.
 */

#ifndef DEFAULTTESTER_H
#define DEFAULTTESTER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <DefaultData.hpp>

class QApplication;

namespace def
{

class DefaultTesterDialog;

/**
 * @author Andrzej Pronobis
 *
 * Tester for the Default.SA
 */
class DefaultTester: public cast::ManagedComponent
{

public:

	/** Where should the query by sent. */
	enum QueryDestination
	{QD_HFC_SERVER, QD_QDL_QUERY_HANDLER};


public:

	/** Constructor. */
	DefaultTester():_qApp(0), _dialog(0) {}

	/** Destructor. */
	virtual ~DefaultTester() {}

	/** Sends a new query. */
	DefaultData::QueryResults sendQuery(std::string query, QueryDestination destination);


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

	/** Id of the QdlQueryHandler component.  */
	std::string _qdlQueryHandlerName;

	/** ICE proxy to the forward chainer server interface. */
	DefaultData::HFCInterfacePrx _hfcInterfacePrx;

	/** ICE proxy to the QdlQueryHandlerInterface. */
	DefaultData::QdlQueryHandlerInterfacePrx _qdlQueryHandlerInterfacePrx;

	/** Qt application. */
	QApplication *_qApp;

	/** QT GUI dialog. */
	DefaultTesterDialog *_dialog;

}; // class DefaultTester
} // namespace def

#endif // DEFAULTTESTER_H



