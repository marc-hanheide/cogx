/**
 * @author Andrzej Pronobis
 *
 * Definition of the ConceptualTester class.
 */

// Conceptual.SA
#include "ConceptualTester.h"
#include "ConceptualTesterDialog.h"
// Qt
#include <QApplication>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new def::ConceptualTester();
	}
}

namespace def
{

using namespace std;
using namespace ConceptualData;


// -------------------------------------------------------
void ConceptualTester::configure(const map<string,string> & _config)
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
void ConceptualTester::start()
{
	// Get the QueryHandler interface proxy
	if (!_queryHandlerName.empty())
		_queryHandlerInterfacePrx= getIceServer<ConceptualData::QueryHandlerInterface>(_queryHandlerName);
}


// -------------------------------------------------------
void ConceptualTester::runComponent()
{
	// Create application
	_qApp = new QApplication(0,0);

	// Start dialog
	_dialog = new ConceptualTesterDialog(this);
	_dialog->exec();

	// Thread safe delete
	ConceptualTesterDialog *dialog=_dialog;
	QApplication *app=_qApp;
	_dialog=0;
	delete dialog;
	_qApp=0;
	delete app;
}


// -------------------------------------------------------
void ConceptualTester::stop()
{
}


// -------------------------------------------------------
DefaultData::DiscreteProbabilityDistribution ConceptualTester::sendQuery(std::string query)
{
}



} // namespace def
