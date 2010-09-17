/**
 * @author Andrzej Pronobis
 *
 * Definition of the def::Tester class.
 */

// Default.SA
#include "Tester.h"


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
using namespace cast;
using namespace DefaultData;


// -------------------------------------------------------
void Tester::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	// Forward chainer server name
	if((it = _config.find("--hfcserver")) != _config.end())
	{
		_hfcServerName = it->second;
	}

	// QueryHandler name
	if((it = _config.find("--queryhandler")) != _config.end())
	{
		_queryHandlerName = it->second;
	}

	log("Configuration parameters:");
	log("-> HFCServer Name: %s", _hfcServerName.c_str());
	log("-> QueryHandler Name: %s", _queryHandlerName.c_str());
}


// -------------------------------------------------------
void Tester::start()
{
	// Get the HFCServer interface proxy
	try
	{
		_hfcInterfacePrx =
				getIceServer<comadata::HFCInterface>(_hfcServerName);
		_hfcServerAvailable = true;
	}
	catch (...)
	{}

	// Get the QueryHandler interface proxy
	try
	{
		_queryHandlerServerInterfacePrx =
				getIceServer<DefaultData::QueryHandlerServerInterface>(_queryHandlerName);
		_queryHandlerAvailable = true;
	}
	catch (...)
	{}
}


// -------------------------------------------------------
void Tester::runComponent()
{
}


// -------------------------------------------------------
void Tester::stop()
{
}


// -------------------------------------------------------
comadata::QueryResults Tester::sendHFCServerQuery(const std::string &query) const
{
	if (_hfcServerAvailable)
		return _hfcInterfacePrx->querySelect(query);
	else
		return comadata::QueryResults();
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution Tester::sendQueryHandlerQuery(const std::string &query) const
{
	if (_queryHandlerAvailable)
		return _queryHandlerServerInterfacePrx->query(query);
	else
		return SpatialProbabilities::ProbabilityDistribution();
}






} // namespace def
