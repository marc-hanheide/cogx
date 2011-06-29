/**
 * CategoricalAppearanceIntegrator class.
 * \file CategoricalAppearanceIntegrator
 * \author Andrzej Pronobis
 */

// Categorical.SA
#include "AppearanceIntegrator.h"
#include "shared/ConfigFile.h"
#include "OutputsCache.h"
#include "SpatialProperties.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
// Std
#include <exception>
#include <math.h>

using namespace std;
using namespace cast;
using namespace categorical;
using boost::bad_lexical_cast;
using boost::lexical_cast;
using namespace boost;

// ------------------------------------------------------
extern "C"
{
cast::interfaces::CASTComponentPtr newComponent()
{
	return new CategoricalAppearanceIntegrator();
}
}



// ------------------------------------------------------
CategoricalAppearanceIntegrator::CategoricalAppearanceIntegrator(): _cfgGroup("AppearanceIntegrator")
{
	_resultsQueue.clear();
	_odometryQueue.clear();

	_previousPlaceId = -1;

	pthread_cond_init(&_dataSignalCond, 0);
	pthread_mutex_init(&_dataSignalMutex, 0);
	pthread_mutex_init(&_intrDataMutex, 0);
}


// ------------------------------------------------------
CategoricalAppearanceIntegrator::~CategoricalAppearanceIntegrator()
{
	pthread_cond_destroy(&_dataSignalCond);
	pthread_mutex_destroy(&_dataSignalMutex);
	pthread_mutex_destroy(&_intrDataMutex);

	delete _nodeCache;
}


// ------------------------------------------------------
void CategoricalAppearanceIntegrator::configure(const map<string,string> &config)
{
	// Read cmd line options
	map<string, string>::const_iterator it = config.find("--config");
	string configFile;
	if (it!=config.end())
		configFile = it->second;
	string labelFile;

	if((it = config.find("--placemanager")) != config.end())
	{
		_placeManagerName = it->second;
	}

	// Read config file
	ConfigFile cf;
	if (configFile.empty())
		println("No configuration file provided. Using defaults instead.");
	else
	{
		if (!cf.read(configFile))
			throw(CASTException(exceptionMessage(__HERE__, "Unable to load configuration file '%s'!", configFile.c_str())));
	}

	// Get configuration from the file
	try
	{
	    _posBinSize=cf.getDoubleValue(_cfgGroup, "PositionBinSize", 0.2);
	    _headBinSize=cf.getDoubleValue(_cfgGroup, "HeadingBinSize", 0.1);
	    _maxPriorOutputsCount=cf.getIntValue(_cfgGroup, "MaxPriorOutputsCount", 50);
	    _cachePoseDecay=cf.getIntValue(_cfgGroup, "CachePoseDecay", true);
	}
	catch(bad_lexical_cast &)
	{
		throw(CASTException(exceptionMessage(__HERE__, "Incorrect item value in the config file '%s'!", configFile.c_str())));
	}

	// Print the configuration
	log("Configuration:");
	log("-> Position cache bin size: %f", _posBinSize);
	log("-> Heading cache bin size: %f", _headBinSize);
	log("-> Max. prior outputs count: %d", _maxPriorOutputsCount);
	log("-> No. of traveled cache bins after which pose information is forgotten: %d", _cachePoseDecay);
}


// ------------------------------------------------------
void CategoricalAppearanceIntegrator::start()
{
	// Add change filters
	addChangeFilter(createLocalTypeFilter<CategoricalData::VisualResults>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<CategoricalAppearanceIntegrator>(this,
					&CategoricalAppearanceIntegrator::newVisualResults));

	addChangeFilter(createLocalTypeFilter<CategoricalData::Odometry>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<CategoricalAppearanceIntegrator>(this,
					&CategoricalAppearanceIntegrator::newOdometry));

	_nodeCache = new OutputsCache(_posBinSize, _headBinSize);



	// Get the QueryHandler interface proxy
	try
	{
		_placeInterfacePrx =
				getIceServer<FrontierInterface::PlaceInterface>(_placeManagerName);
	}
	catch (...)
	{}


	debug("Started!");
}


// ------------------------------------------------------
void CategoricalAppearanceIntegrator::stop()
{
	debug("Stop!");
}


// ------------------------------------------------------
void CategoricalAppearanceIntegrator::newOdometry(const cast::cdl::WorkingMemoryChange & change)
{
	pthread_mutex_lock(&_dataSignalMutex);

	// Grab data
	sched_yield();
	shared_ptr<CASTData<CategoricalData::Odometry> > resultsCast =
			getWorkingMemoryEntry<CategoricalData::Odometry>(change.address);
	if (!resultsCast)
	{
		error("Cannot get Odometry from WM!");
		return;
	}
	const CategoricalData::OdometryPtr scan = resultsCast->getData();

	// Addd to queue
	_odometryQueue.push_back(new CategoricalData::Odometry(*scan));

	sched_yield();

	// Signal
	pthread_cond_signal(&_dataSignalCond);
	pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
void CategoricalAppearanceIntegrator::newVisualResults(const cast::cdl::WorkingMemoryChange & change)
{
	pthread_mutex_lock(&_dataSignalMutex);

	// Grab data
	sched_yield();
	shared_ptr<CASTData<CategoricalData::VisualResults> > resultsCast =
			getWorkingMemoryEntry<CategoricalData::VisualResults>(change.address);
	if (!resultsCast)
	{
		error("Cannot get VisualResults from WM!");
		return;
	}
	const CategoricalData::VisualResultsPtr scan = resultsCast->getData();

	// Addd to queue
	_resultsQueue.push_back(new CategoricalData::VisualResults(*scan));

	sched_yield();

	// Signal
	pthread_cond_signal(&_dataSignalCond);
	pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
bool CategoricalAppearanceIntegrator::readyToAccumulate()
{
	if ( (_resultsQueue.empty()) || (_odometryQueue.empty()) )
		return false;

	if (_resultsQueue.front()->frameNo != _odometryQueue.front()->frameNo)
		return false;

	return true;
}


// ------------------------------------------------------
void CategoricalAppearanceIntegrator::runComponent()
{
	debug("Running...");

	// Run component
	while(isRunning())
	{
// Get current time and add 1sec
timeval tv;
timespec ts;
gettimeofday(&tv, NULL);
ts.tv_sec = tv.tv_sec + 1;
ts.tv_nsec = 0;

		// Wait if necessary
		pthread_mutex_lock(&_dataSignalMutex);
		if (!readyToAccumulate())
			pthread_cond_timedwait(&_dataSignalCond, &_dataSignalMutex, &ts);

		// Handle signal if signal arrived
		if ((!isRunning()) || (!readyToAccumulate()))
			pthread_mutex_unlock(&_dataSignalMutex);
		else
		{
			CategoricalData::VisualResultsPtr results = _resultsQueue.front();
			_resultsQueue.pop_front();
			CategoricalData::OdometryPtr odom = _odometryQueue.front();
			_odometryQueue.pop_front();

			pthread_mutex_unlock(&_dataSignalMutex);

			debug("Grabbed results and matching odometry. Accumulating...");

			sched_yield();


			if ( (results->status==CategoricalData::DsValid) && (odom->status==CategoricalData::DsValid) )
			{
				// Get the current place id
				SpatialData::PlacePtr curPlace ;
				try
				{
					curPlace = _placeInterfacePrx->getCurrentPlace();
				}
				catch(...)
				{
					log("Could not get the current place!!");
					curPlace=0;
				}

				if (curPlace)
				{
					log("%d", curPlace->id);


					// Lock the internal data
					pthread_mutex_lock(&_intrDataMutex);

					// Create empty prior for that place if it does not yet exist
					if (_priors.find(curPlace->id) == _priors.end())
					{
						Prior p;
						p.count = 0;
						_priors[curPlace->id] = p;
					}

					if (_previousPlaceId!=curPlace->id)
					{	// We changed place
						// Check if we have something in the cache
						if (!_nodeCache->isEmpty())
						{
							// Integrate the cache with prior
							unsigned int nodeOutputCount;
							CategoricalData::ClassifierOutputs outputs;
							_nodeCache->getAccumulatedOutputs(outputs, nodeOutputCount);

							integrateWithPrior(_priors[_previousPlaceId].outputs,
									_priors[_previousPlaceId].count, outputs, nodeOutputCount);
						}
						// Clear cache
						_nodeCache->clear();
						_previousPlaceId = curPlace->id;
					}

					// Add the outputs to the caches
					_nodeCache->addOutputs(odom->odometryBuffer.odompose[0].x, odom->odometryBuffer.odompose[0].y,
							odom->odometryBuffer.odompose[0].theta, results->outputs);


					// Get the accumulated outputs
					unsigned int nodeOutputCount;
					CategoricalData::ClassifierOutputs outputs;
					_nodeCache->getAccumulatedOutputs(outputs, nodeOutputCount);

					// Get outputs combined with prior
					CategoricalData::ClassifierOutputs outputsWithPrior;
					outputsWithPrior = combineWithPrior(_priors[curPlace->id].outputs,
							_priors[curPlace->id].count, outputs, nodeOutputCount);

					// Print for debugging.
					//				for (unsigned int i=0; i<outputs.size(); ++i)
					//				{
					//					if (!_priors[curPlace->id].outputs.empty())
					//						log("place %d, name=%s output=%f outputWithPrior=%f, prior=%f", curPlace->id, outputs[i].name.c_str(),
					//													outputs[i].value, outputsWithPrior[i].value, _priors[curPlace->id].outputs[i].value);
					//					else
					//						log("place %d, name=%s output=%f outputWithPrior=%f", curPlace->id, outputs[i].name.c_str(),
					//								outputs[i].value, outputsWithPrior[i].value);
					//				}
					//				_nodeCache->print();
					//				_nodeCache->printAccumulatedOutputs();
					//				_nodeCache->printNormalizedAccumulatedOutputs();


					// Now, turn the outputs into "probabilities" or rather potentials
					vector<double> potentials;
					potentials.resize(outputsWithPrior.size());
					double sum = 0;
					for (unsigned int i=0; i<outputs.size(); ++i)
					{
						potentials[i] = outputsWithPrior[i].value;
						potentials[i] /= (_priors[curPlace->id].count + nodeOutputCount);
						potentials[i] = 1.0 / (1.0 + exp(-2.0*potentials[i]));

						debug("place %d, name=%s potential=%lf", curPlace->id, outputs[i].name.c_str(),
								potentials[i]);
						sum+=potentials[i];
					}

					// Normalize the potentials
					for (unsigned int i=0; i<potentials.size(); ++i)
					{
						potentials[i]/=sum;
					}

					// Output the property
					SpatialProperties::RoomAppearancePlacePropertyPtr propertyPtr =
							new SpatialProperties::RoomAppearancePlaceProperty();
					propertyPtr->placeId = curPlace->id;
					SpatialProperties::DiscreteProbabilityDistributionPtr probDistPtr = new
							SpatialProperties::DiscreteProbabilityDistribution();

					SpatialProperties::StringValuePtr mapStrVal =
							new SpatialProperties::StringValue();
					double mapPotential = -1.0;

					for (unsigned int i=0; i<potentials.size(); ++i)
					{
						SpatialProperties::ValueProbabilityPair vpp;
						vpp.probability = potentials[i];
						SpatialProperties::StringValuePtr strVal =
								new SpatialProperties::StringValue();
						strVal->value = outputs[i].name;
						vpp.value = strVal;
						if (potentials[i]>mapPotential)
						{
							mapStrVal->value = outputs[i].name;
							mapPotential = potentials[i];
						}
						probDistPtr->data.push_back(vpp);
					}

					propertyPtr->distribution = probDistPtr;
					propertyPtr->mapValue = mapStrVal;
					propertyPtr->mapValueReliable = true;

					// Generate id if it's not there yet
					if (_placeAppearancePropertyIds.find(curPlace->id) == _placeAppearancePropertyIds.end())
					{
						_placeAppearancePropertyIds[curPlace->id] = newDataID();
						addToWorkingMemory<SpatialProperties::RoomAppearancePlaceProperty>(
								_placeAppearancePropertyIds[curPlace->id], propertyPtr);
					}
					else
						overwriteWorkingMemory<SpatialProperties::RoomAppearancePlaceProperty>(
								_placeAppearancePropertyIds[curPlace->id], propertyPtr);

					// Unock the internal data
					pthread_mutex_unlock(&_intrDataMutex);
				} // cur place valid
			} // valid
			else
			{
				// Wait a short while not to loop
				usleep(50000);

				continue;
			}

			sched_yield();

		} // if
	} // while

	debug("Completed!");
}



// ------------------------------------------------------
void CategoricalAppearanceIntegrator::integrateWithPrior(CategoricalData::ClassifierOutputs &priorOutputs,
		unsigned int &priorOutputsCount, const CategoricalData::ClassifierOutputs &newOutputs,
		unsigned int newOutputsCount)
{
  // If there is nothing in the newOutputs, leave the prior as it was.
  if (newOutputsCount==0)
  {
    return;
  }

  // If there is nothing in the prior, just replace it
  if (priorOutputsCount==0)
  {
    priorOutputs=newOutputs;
    priorOutputsCount=newOutputsCount;
    return;
  }

  // Check if the prior and the outputs match, if not, raise exception
  int priorLength = priorOutputs.size();
  int newLength = newOutputs.size();
  if (priorLength != newLength)
    throw CASTException(exceptionMessage(__HERE__, "Outputs cannot be integrated with the prior as they do not match (%d, %d)!",
                        priorLength, newLength));

  //
  double priorOutputsCountOrig = priorOutputsCount;
  if (priorOutputsCount>_maxPriorOutputsCount)
  {
    priorOutputsCount=_maxPriorOutputsCount;
  }

  // Now accumulate and check
  for (int i=0; i<priorLength; ++i)
  {
    if (string(priorOutputs[i].name) != string(newOutputs[i].name))
    {
      throw CASTException(exceptionMessage(__HERE__, "Outputs cannot be integrated with the prior as they do not match (%s, %s)!",
                          string(priorOutputs[i].name).c_str(), string(newOutputs[i].name).c_str()));
    }
    priorOutputs[i].value/=priorOutputsCountOrig;
    priorOutputs[i].value*=static_cast<double>(priorOutputsCount);
    priorOutputs[i].value+=newOutputs[i].value;
  }
  priorOutputsCount+=newOutputsCount;
}




// ------------------------------------------------------
CategoricalData::ClassifierOutputs CategoricalAppearanceIntegrator::combineWithPrior(const CategoricalData::ClassifierOutputs &priorOutputs,
		unsigned int &priorOutputsCount, const CategoricalData::ClassifierOutputs &newOutputs,
		unsigned int newOutputsCount)
{
  // If there is nothing in the newOutputs, leave it as it was
  if (newOutputsCount==0)
  {
    return newOutputs;
  }

  // If there is nothing in the prior, don't combine
  if (priorOutputsCount==0)
  {
    return newOutputs;
  }

  // Check if the prior and the outputs match, if not, raise exception
  int priorLength = priorOutputs.size();
  int newLength = newOutputs.size();
  if (priorLength != newLength)
    throw CASTException(exceptionMessage(__HERE__, "Outputs cannot be integrated with the prior as they do not match (%d, %d)!",
                        priorLength, newLength));

  //
  double priorOutputsCountOrig = priorOutputsCount;
  if (priorOutputsCount>_maxPriorOutputsCount)
  {
    priorOutputsCount=_maxPriorOutputsCount;
  }

  CategoricalData::ClassifierOutputs out;
  out.resize(priorLength);

  // Combine prior with the new data
  for (int i=0; i<priorLength; ++i)
  {
    if (string(priorOutputs[i].name) != string(newOutputs[i].name))
    {
      throw CASTException(exceptionMessage(__HERE__, "Outputs cannot be integrated with the prior as they do not match (%s, %s)!",
                          string(priorOutputs[i].name).c_str(), string(newOutputs[i].name).c_str()));
    }
    out[i].name = newOutputs[i].name;
    out[i].value = priorOutputs[i].value / priorOutputsCountOrig;
    out[i].value*=static_cast<double>(priorOutputsCount);
    out[i].value+=newOutputs[i].value;
  }

  return out;
}
