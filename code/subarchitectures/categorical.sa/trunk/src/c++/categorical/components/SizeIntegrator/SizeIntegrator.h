/**
 * SizeIntegrator class.
 * \file SizeIntegrator
 * \author Andrzej Pronobis
 */

#ifndef __CATEGORICAL_SIZE_INTEGRATOR__
#define __CATEGORICAL_SIZE_INTEGRATOR__

#include <cast/architecture/ManagedComponent.hpp>
#include <CategoricalData.hpp>
#include "FrontierInterface.hpp"

namespace categorical
{
  class OutputsCache;
}

/**
 * Implements the LaserProcessor component
 */
class CategoricalSizeIntegrator: public cast::ManagedComponent
{
public: // Component management

	/** Constructor. */
	CategoricalSizeIntegrator();

	/** Destructor. */
	virtual ~CategoricalSizeIntegrator();

	/** Handles component configuration. */
	virtual void configure(const std::map<std::string,std::string> &config);

	/** Main thread of the component. */
	virtual void runComponent();

	/** Invoked on start. */
	virtual void start();

	/** Invoked on stop. */
	virtual void stop();


private:

	void newLaserResults(const cast::cdl::WorkingMemoryChange & change);
	void newOdometry(const cast::cdl::WorkingMemoryChange & change);
	void newSizePlaceProperty(const cast::cdl::WorkingMemoryChange &change);

	bool readyToAccumulate();
	void ensureQueueIntegrity();

	void integrateWithPrior(CategoricalData::ClassifierOutputs &priorOutputs,
			unsigned int &priorOutputsCount, const CategoricalData::ClassifierOutputs &newOutputs,
			unsigned int newOutputsCount);

	CategoricalData::ClassifierOutputs combineWithPrior(const CategoricalData::ClassifierOutputs &priorOutputs,
			unsigned int &priorOutputsCount, const CategoricalData::ClassifierOutputs &newOutputs,
			unsigned int newOutputsCount);

private:

	std::list<CategoricalData::LaserResultsPtr> _resultsQueue;
	std::list<CategoricalData::OdometryPtr> _odometryQueue;

	pthread_cond_t _dataSignalCond;
	pthread_mutex_t _dataSignalMutex;
	/** Mutex used for accessing caches and internal data. */
	pthread_mutex_t _intrDataMutex;


	categorical::OutputsCache *_nodeCache;

	double _posBinSize;
	double _headBinSize;

	/** Max. influence of prior. */
	unsigned int _maxPriorOutputsCount;

	/** No. of traveled cache bins after which pose information is forgotten.
	 * <0 means "never".*/
	int _cachePoseDecay;

	FrontierInterface::PlaceInterfacePrx _placeInterfacePrx;

	std::string _placeManagerName;

private:

	/** Name of the config file group. */
	const std::string _cfgGroup;

	struct Prior
	{
		CategoricalData::ClassifierOutputs outputs;
		unsigned int count;
	};

	std::map<int, Prior> _priors;


	int _previousPlaceId;

	/** Ids of the property structs on the working memory. */
	std::map<int, std::string> _placeSizePropertyIds;

};


#endif

