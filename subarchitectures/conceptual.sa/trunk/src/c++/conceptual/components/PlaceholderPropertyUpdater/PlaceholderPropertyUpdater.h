/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::PlaceholderPropertyUpdater class.
 */

#ifndef CONCEPTUAL_PLACEHOLDERPROPERTYUPDATER_H
#define CONCEPTUAL_PLACEHOLDERPROPERTYUPDATER_H

#include <cast/architecture/ManagedComponent.hpp>
#include "ConceptualData.hpp"
#include "DefaultData.hpp"
#include "SpatialProbabilities.hpp"
#include "SpatialProperties.hpp"

namespace conceptual
{

/**
 * @author Andrzej Pronobis
 *
 * Updates the values of placeholder properties on the spatial.sa WM.
 */
class PlaceholderPropertyUpdater: public cast::ManagedComponent
{

public:

	/** Constructor. */
	PlaceholderPropertyUpdater();

	/** Destructor. */
	virtual ~PlaceholderPropertyUpdater();


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

	/** World state changed, infer and then update the coma room structs. */
	void worldStateChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	void placeChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Updates or creates the property on Spatial.SA working memory. */
	void updateRoomCategoryPlaceholderProperty(int placeholderId,
			std::string category, const SpatialProbabilities::ProbabilityDistribution &pd);

	/** Returns WM id of property based on the info stored in _placeholderProperties. */
	cast::cdl::WorkingMemoryAddress getRoomCategoryPlaceholderPropertyWmAddress(
			int placeholderId, std::string category);

	/** Updates the probability distribution in the property struct. */
	void setRoomCategoryPlaceholderPropertyDistribution(
			SpatialProperties::RoomCategoryPlaceholderPropertyPtr propertyPtr,
			const SpatialProbabilities::ProbabilityDistribution &pd);

	bool placeholderExists(int id);


private:

	  pthread_cond_t _worldStateChangedSignalCond;
	  pthread_mutex_t _worldStateChangedSignalMutex;

	  /** True if the world state has changed since the last time we checked. */
	  bool _worldStateChanged;

	  /** Vector of Placeholder Ids. */
	  std::vector<int> _placeholderIds;

	  /** Name of the QueryHandler component.  */
	  std::string _queryHandlerName;

	  /** Name of the DefaultChainGraphInferencer component.  */
	  std::string _defaultChainGraphInferencerName;

	  /** Names of all room categories. */
	  DefaultData::StringSeq _roomCategories;

	  /** ICE proxy to the QueryHandlerInterface. */
	  ConceptualData::QueryHandlerServerInterfacePrx _queryHandlerServerInterfacePrx;

	  /** ICE proxy to the DefaultData::ChainGraphInferencerInterface. */
	  DefaultData::ChainGraphInferencerServerInterfacePrx _defaultChainGraphInferencerServerInterfacePrx;

	  /** Map of place wmAddress -> place id */
	  std::map<cast::cdl::WorkingMemoryAddress, int> _placeWmAddressMap;



private:

	  struct RoomCategoryPlaceholderPropertyInfo
	  {
		  cast::cdl::WorkingMemoryAddress wmAddress;
		  int placeholderId;
		  std::string category;
	  };

	  /** Map containing all the placeholder properties created by the component. */
	  std::list<RoomCategoryPlaceholderPropertyInfo> _placeholderProperties;

}; // class PlaceholderPropertyUpdater
} // namespace

#endif // CONCEPTUAL_PLACEHOLDERPROPERTYUPDATER_H



