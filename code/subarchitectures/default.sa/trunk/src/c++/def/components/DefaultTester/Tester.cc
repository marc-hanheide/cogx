/**
 * @author Andrzej Pronobis
 *
 * Definition of the def::Tester class.
 */

// Default.SA
#include "Tester.h"
#include "TesterDialog.h"
// Qt
#include <QApplication>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new def::Tester();
	}
}

namespace def
{

using namespace std;
using namespace DefaultData;


// -------------------------------------------------------
void Tester::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	// Forward chainer server name
	if((it = _config.find("--test-hfc")) != _config.end())
	{
		_hfcServerName = it->second;
	}

	// QueryHandler name
	if((it = _config.find("--test-qh")) != _config.end())
	{
		_queryHandlerName = it->second;
	}

	log("Configuration parameters:");
	log("-> Test HFCServer: %s", (_hfcServerName.empty())?"No":"Yes");
	log("-> HFCServer Name: %s", _hfcServerName.c_str());
	log("-> Test QueryHandler: %s", (_queryHandlerName.empty())?"No":"Yes");
	log("-> QueryHandler Name: %s", _queryHandlerName.c_str());
}


// -------------------------------------------------------
void Tester::start()
{
	// Get the forward chainer interface proxy
	if (!_hfcServerName.empty())
		_hfcInterfacePrx = getIceServer<DefaultData::HFCInterface>(_hfcServerName);
	// Get the QueryHandler interface proxy
	if (!_queryHandlerName.empty())
		_queryHandlerInterfacePrx= getIceServer<DefaultData::QueryHandlerInterface>(_queryHandlerName);
}


// -------------------------------------------------------
void Tester::runComponent()
{
	// Create application
	_qApp = new QApplication(0,0);

	// Start dialog
	_dialog = new TesterDialog(this, !_hfcServerName.empty(), !_queryHandlerName.empty());
	_dialog->exec();

	// Thread safe delete
	TesterDialog *dialog=_dialog;
	QApplication *app=_qApp;
	_dialog=0;
	delete dialog;
	_qApp=0;
	delete app;
}


// -------------------------------------------------------
void Tester::stop()
{
}


// -------------------------------------------------------
QdlQueryResults Tester::sendQuery(std::string query, QueryDestination destination)
{
	println("Sending query %s", query.c_str());
	return _hfcInterfacePrx->querySelect(query);
}



} // namespace def
