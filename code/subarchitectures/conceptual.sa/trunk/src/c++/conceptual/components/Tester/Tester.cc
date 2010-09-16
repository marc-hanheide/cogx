/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::Tester class.
 */

// Conceptual.SA
#include "Tester.h"
#include "TesterDialog.h"
// Qt
#include <QApplication>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::Tester();
	}
}

namespace conceptual
{

using namespace std;
using namespace ConceptualData;


// -------------------------------------------------------
void Tester::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--test-qh")) != _config.end())
	{
		_queryHandlerName = it->second;
	}

	log("Configuration parameters:");
	log("-> Test QueryHandler: %s", (_queryHandlerName.empty())?"No":"Yes");
	log("-> QueryHandler Name: %s", _queryHandlerName.c_str());
}


// -------------------------------------------------------
void Tester::start()
{
	// Get the QueryHandler interface proxy
	if (!_queryHandlerName.empty())
		_queryHandlerServerInterfacePrx= getIceServer<ConceptualData::QueryHandlerServerInterface>(_queryHandlerName);
}


// -------------------------------------------------------
void Tester::runComponent()
{
	// Create application
	_qApp = new QApplication(0,0);

	// Start dialog
	_dialog = new TesterDialog(this);
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
DefaultData::DiscreteProbabilityDistribution Tester::sendQuery(std::string query)
{
}



} // namespace def
