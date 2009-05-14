#ifndef BINDING_COMPARATOR_CACHE_H_
#define BINDING_COMPARATOR_CACHE_H_

//#include "utils/BindingScoreUtils.hpp"
//#include "utils/BindingUtils.hpp"
#include "binding/feature-utils/AbstractFeature.hpp"
#include "balt/core/StringMap.hpp"
#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>
#include <string>

namespace Binding {

//forward decl
class BindingScorer;
class ComparatorCacheCheck;

/// Calculates scores between proxies and instances
class ComparatorCache {
  
  
  friend class BindingScorer;
  friend class ComparatorCacheCheck;

  //CASTDataCache<BindingData::FeatureComparison> m_comparisonCache;
  /// stores all computed comparisons m_comparison[_combinedID(featureID,unionID)]
  cast::StringMap<boost::logic::tribool>::map m_internalCache;
  BindingScorer& m_bindingScorer;

private:

  ComparatorCache(BindingScorer& _bindingScorer)
    : m_bindingScorer(_bindingScorer) {}
  
  /// calls _get(...)
  boost::logic::tribool get(const AbstractFeature& _proxyFeaturePtr, 
			    const AbstractFeature& _unionFeaturePtr,
			    const std::string& _proxyID);

  /// returns the result of a comparison between two features
  boost::logic::tribool _get(const AbstractFeature& _proxyFeaturePtr, 
			     const AbstractFeature& _unionFeaturePtr,
			     const std::string& _proxyID);

  /// sets a comparison result in the internal cache. Called by the
  /// bindingscorer as the result of events.
  void set(const std::string& _comparisonID);
  
};

} // namespace Binding

#endif // BINDING_COMPARATOR_CACHE_H_
