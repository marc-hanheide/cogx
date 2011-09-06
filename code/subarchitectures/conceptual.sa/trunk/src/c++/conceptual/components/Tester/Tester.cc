/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::Tester class.
 */

// Conceptual.SA
#include "Tester.h"
#include "MainDialog.h"
#include "ConceptualWidget.h"
#include "SpatialProbabilities.hpp"
#include "NavWidget.h"
#include "CategoricalWidget.h"
#include "DemoWidget.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Qt, std
#include <QApplication>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

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
using namespace cast;
using namespace ConceptualData;
using namespace boost;


// -------------------------------------------------------
void Tester::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--queryhandler")) != _config.end())
		_queryHandlerName = it->second;
	if((it = _config.find("--chaingraphinferencer")) != _config.end())
		_chainGraphInferencerName = it->second;
	if((it = _config.find("--defaultchaingraphinferencer")) != _config.end())
		_defaultChainGraphInferencerName = it->second;
	if((it = _config.find("--placemanager")) != _config.end())
		_placeManagerName = it->second;
	_saveEvents = (_config.find("--save-events") != _config.end());
	if((it = _config.find("--visualized-objects")) != _config.end())
		boost::split(_visualizedObjects, it->second, is_any_of(", "));
	sort (_visualizedObjects.begin(), _visualizedObjects.end());

	log("Configuration parameters:");
	log("-> QueryHandler name: %s", _queryHandlerName.c_str());
	log("-> ChainGraphInferencer name: %s", _chainGraphInferencerName.c_str());
	log("-> DefaultChainGraphInferencer name: %s", _defaultChainGraphInferencerName.c_str());
	log("-> PlaceManager name: %s", _placeManagerName.c_str());
}


// -------------------------------------------------------
void Tester::start()
{
	try
	{
		// Get the QueryHandler interface proxy
		_queryHandlerServerInterfacePrx =
				getIceServer<ConceptualData::QueryHandlerServerInterface>(_queryHandlerName);
		_queryHandlerAvailable = true;
	}
	catch(...)
	{}

	try
	{
		// Get the ChainGraphInferencer interface proxy
		_chainGraphTestingServerInterfacePrx =
				getIceServer<ConceptualData::ChainGraphTestingServerInterface>(_chainGraphInferencerName);
		_chainGraphInferencerAvailable = true;
	}
	catch(...)
	{}

	try
	{
		// Get the DefaultChainGraphInferencer interface proxy
		_defaultChainGraphInferencerServerInterfacePrx =
				getIceServer<DefaultData::ChainGraphInferencerServerInterface>(_defaultChainGraphInferencerName);
		_defaultChainGraphInferencerAvailable = true;
	}
	catch(...)
	{}

	try
	{
		// Get the PlaceInterface interface proxy
		_placeInterfacePrx =
				getIceServer<FrontierInterface::PlaceInterface>(_placeManagerName);
		_placeManagerAvailable = true;
	}
	catch(...)
	{}

	// Change filters
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::worldStateChanged));
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::worldStateChanged));
	addChangeFilter(createGlobalTypeFilter<SpatialData::NavCommand>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newNavCommand));
	addChangeFilter(createGlobalTypeFilter<SpatialData::NavCommand>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newNavCommand));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::LaserScan>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newLaserScan));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::LaserScan>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newLaserScan));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::Odometry>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newOdometry));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::Odometry>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newOdometry));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::Image>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newImage));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::Image>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newImage));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::LaserResults>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newLaserResults));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::LaserResults>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newLaserResults));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::VisualResults>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newVisualResults));
	addChangeFilter(createGlobalTypeFilter<CategoricalData::VisualResults>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::newVisualResults));
}


// -------------------------------------------------------
void Tester::runComponent()
{
	// Get default knowledge
	_objectPropertyVariables =
		_defaultChainGraphInferencerServerInterfacePrx->getObjectPropertyVariables();
	_objectCategories =
		_defaultChainGraphInferencerServerInterfacePrx->getObjectCategories();
	_roomCategories =
		_defaultChainGraphInferencerServerInterfacePrx->getRoomCategories();
	_shapes =
		_defaultChainGraphInferencerServerInterfacePrx->getShapes();
	_sizes =
		_defaultChainGraphInferencerServerInterfacePrx->getSizes();
	_appearances =
		_defaultChainGraphInferencerServerInterfacePrx->getAppearances();
	_humanAssertions =
		_defaultChainGraphInferencerServerInterfacePrx->getHumanAssertions();
	// Sort room categories for nice display
	sort (_roomCategories.begin(), _roomCategories.end());
	sort (_shapes.begin(), _shapes.end());
	sort (_sizes.begin(), _sizes.end());
	sort (_appearances.begin(), _appearances.end());

	QCoreApplication *app = QApplication::instance();
	if (!app) {
		int argc=0;
		app = new QApplication(argc,new char*[0]);
	}
	MainDialog *mainDialog = new MainDialog(this);
	_mainDialog = mainDialog;
	mainDialog->exec();
	_mainDialog=0;
	delete mainDialog;
}


// -------------------------------------------------------
void Tester::stop()
{
}


// -------------------------------------------------------
ConceptualData::ProbabilityDistributions Tester::sendQueryHandlerQuery(const std::string &query, bool imaginary, bool factor)
{
	if (_queryHandlerAvailable)
	{
		if (imaginary)
			return _queryHandlerServerInterfacePrx->imaginaryQuery(query);
		else if (factor)
			return _queryHandlerServerInterfacePrx->factorQuery(query);
		else
			return _queryHandlerServerInterfacePrx->query(query);
	}
	else
		return ConceptualData::ProbabilityDistributions();
}


// -------------------------------------------------------
void Tester::worldStateChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	try
	{
		ConceptualData::WorldStatePtr worldStatePtr;
		worldStatePtr = getMemoryEntry<ConceptualData::WorldState>(wmChange.address);

		if (worldStatePtr)
		{
			log("New worldstate for event %d", worldStatePtr->lastEvents.back().type);
			if (_mainDialog)
			{
				_mainDialog->getConceptualWidget()->newWorldState(worldStatePtr);
				_mainDialog->getDemoWidget()->newWorldState(worldStatePtr);
			}
		}
	}
	catch(CASTException &e)
	{
		log("Exception while reading world state from the WM!");
	}
}


// -------------------------------------------------------
ConceptualData::VariableInfos Tester::getChainGraphVariables()
{
	if (_chainGraphInferencerAvailable)
	{
		return _chainGraphTestingServerInterfacePrx->getVariables();
	}
	else
		return ConceptualData::VariableInfos();
}


// -------------------------------------------------------
ConceptualData::FactorInfos Tester::getChainGraphFactors()
{
	if (_chainGraphInferencerAvailable)
	{
		return _chainGraphTestingServerInterfacePrx->getFactors();
	}
	else
		return ConceptualData::FactorInfos();
}


// -------------------------------------------------------
int Tester::getCurrentPlace()
{
	if (_placeManagerAvailable)
	{
		SpatialData::PlacePtr pl= _placeInterfacePrx->getCurrentPlace();
		if (pl)
			return pl->id;
		else
			return -1;
	}
	else
		return -1;
}


// -------------------------------------------------------
void Tester::postNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype)
{
	SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
	cmd->prio = SpatialData::URGENT;
	cmd->cmd = cmdtype;
	cmd->pose.resize(3);
	cmd->pose[0] = position.getX();
	cmd->pose[1] = position.getY();
	cmd->pose[2] = position.getTheta();
	cmd->tolerance.resize(1);
	cmd->tolerance[0] = 0.1;
	cmd->status = SpatialData::NONE;
	cmd->comp = SpatialData::COMMANDPENDING;

	string id(newDataID());
	addToWorkingMemory<SpatialData::NavCommand> (id, "spatial.sa", cmd);
}


// -------------------------------------------------------
void Tester::newNavCommand(const cast::cdl::WorkingMemoryChange &wmChange)
{
	try
	{
		SpatialData::NavCommandPtr navCommandPtr;
		lockEntry(wmChange.address, cast::cdl::LOCKEDOD);
		navCommandPtr = getMemoryEntry<SpatialData::NavCommand>(wmChange.address);
		if (holdsLock(wmChange.address.id, wmChange.address.subarchitecture)) {
			unlockEntry(wmChange.address.id, wmChange.address.subarchitecture);
		}

		if (navCommandPtr)
		{
			if (_mainDialog)
				_mainDialog->getNavWidget()->newNavCommand(navCommandPtr);
		}
	}
	catch(...)
	{
		log("Exception while reading nav command from the WM!");
		if (holdsLock(wmChange.address.id, wmChange.address.subarchitecture)) {
			unlockEntry(wmChange.address.id, wmChange.address.subarchitecture);
		}
	}
}



// -------------------------------------------------------
void Tester::newLaserScan(const cast::cdl::WorkingMemoryChange &wmChange)
{
	try
	{
		CategoricalData::LaserScanPtr laserScanPtr;
		laserScanPtr = getMemoryEntry<CategoricalData::LaserScan>(wmChange.address);

		if (laserScanPtr)
		{
			if (_mainDialog)
			{
				_mainDialog->getCategoricalWidget()->newLaserScan(laserScanPtr);
				_mainDialog->getDemoWidget()->newLaserScan(laserScanPtr);
			}
		}
	}
	catch(...)
	{
		log("Exception while reading from the WM!");
	}
}


// -------------------------------------------------------
void Tester::newOdometry(const cast::cdl::WorkingMemoryChange &wmChange)
{
	try
	{
		CategoricalData::OdometryPtr odometryPtr;
		odometryPtr = getMemoryEntry<CategoricalData::Odometry>(wmChange.address);

		if (odometryPtr)
		{
			if (_mainDialog)
				_mainDialog->getCategoricalWidget()->newOdometry(odometryPtr);
		}
	}
	catch(...)
	{
		log("Exception while reading from the WM!");
	}
}


// -------------------------------------------------------
void Tester::newImage(const cast::cdl::WorkingMemoryChange &wmChange)
{
	try
	{
		CategoricalData::ImagePtr imagePtr;
		imagePtr = getMemoryEntry<CategoricalData::Image>(wmChange.address);

		if (imagePtr)
		{
			if (_mainDialog)
			{
				_mainDialog->getCategoricalWidget()->newImage(imagePtr);
				_mainDialog->getDemoWidget()->newImage(imagePtr);
			}
		}
	}
	catch(...)
	{
		log("Exception while reading from the WM!");
	}
}


// -------------------------------------------------------
void Tester::newLaserResults(const cast::cdl::WorkingMemoryChange &wmChange)
{
	try
	{
		CategoricalData::LaserResultsPtr laserResultsPtr;
		laserResultsPtr = getMemoryEntry<CategoricalData::LaserResults>(wmChange.address);

		if (laserResultsPtr)
		{
			if (_mainDialog)
			{
				_mainDialog->getCategoricalWidget()->newLaserResults(laserResultsPtr);
				_mainDialog->getDemoWidget()->newLaserResults(laserResultsPtr);
			}
		}
	}
	catch(...)
	{
		log("Exception while reading from the WM!");
	}
}


// -------------------------------------------------------
void Tester::newVisualResults(const cast::cdl::WorkingMemoryChange &wmChange)
{
	try
	{
		CategoricalData::VisualResultsPtr visualResultsPtr;
		visualResultsPtr = getMemoryEntry<CategoricalData::VisualResults>(wmChange.address);

		if (visualResultsPtr)
		{
			if (_mainDialog)
			{
				_mainDialog->getCategoricalWidget()->newVisualResults(visualResultsPtr);
				_mainDialog->getDemoWidget()->newVisualResults(visualResultsPtr);
			}
		}
	}
	catch(...)
	{
		log("Exception while reading from the WM!");
	}
}




} // namespace def
