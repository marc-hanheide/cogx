#ifndef BINDING_FEATURE_PROPERTIES_UTILS_H_
#define BINDING_FEATURE_PROPERTIES_UTILS_H_

#include <map>
#include <set>
#include <binding/idl/BindingFeatures.hh>
#include <stdexcept> 
#include <iostream> 
#include <boost/logic/tribool.hpp> 
#include "cast/core/CASTCore.hpp"

namespace Binding {
class BindingFeatureOntology;

/// Collects all properties of features into one class. These
/// properties must be satisfied in the feature sets of binding
/// proxies, otherwise an exception will be thrown.
class FeatureProperties {
public:
  FeatureProperties() 
    : isInvariant(false),
      isSimplex(false),
      isDemanded(false){}//,isImplemented(false) {};
  /// an invariant feature should be constant for an binding union
  /// if true, then the feature type is considered
  /// invariant enough to be part of the binding
  /// score. Visibility is an example of a feature that is not
  /// invariant. Colour is an example of a feature that is invariant.
  bool isInvariant;
  /// if true, then the feature type can have only one value 
  bool isSimplex;
  /// A feature that is demanded must be defined in the proxy
  bool isDemanded;
  /// A property for features that are being defined in the binding
  /// IDL etc but are not yet meant to be used.
//  bool isImplemented;

  friend std::ostream& operator<<(std::ostream&,const FeatureProperties&);

private:
  friend class BindingFeatureOntology;

  /// contains a set of features with which this feature can be
  /// compared (within the binder subarch itself). OBS! This
  /// relation is not necessarily symmetric and is meant to
  /// represent whether a feature in a proxy is comparable in a
  /// feature in an binding union. A feature A can therefor be
  /// compared with a feature B during scoring, but the feature B
  /// does not need to be compared to feature A.  If you want two
  /// features to be comparable "in both directions", you need to
  /// declare them both comparable in both feature declarations.
  std::set<std::string> comparableInternally;
  /// conatins a set of features that are comparable by some other
  /// subarchitecture. This will result in a \p
  /// BindingData::FeatureComparison being added to WM and this will
  /// then (hopefully) be overwritten by the result of the external
  /// evaluation. The \p comparableExternally set may overlap the \p
  /// comparableInternally set in which case the result of the
  /// internal comparison will be used for scoring until the external
  /// result is received.
  std::set<std::string> comparableExternally;

  /// returns true if \p _feature is a member of either \p
  /// comparableExternally or \p comparableInternally
  bool comparable(const std::string& _feature) const {
    return comparableExternally.find(_feature) != comparableExternally.end()
      || comparableInternally.find(_feature) != comparableInternally.end();
  }
};


/// specialize if you want a more specialized dot label
template<class T>
std::string toDotLabel(const T& _t) {
  if(negated(_t)) 
    return std::string("~") + toString(_t);
  return toString(_t);
}

inline
std::string 
toString(const CORBA::String_member& _m)
{
  return std::string(_m);
}

inline
std::string 
toDotLabel(const CORBA::String_member& _m)
{
  return std::string("\\\"") + std::string(_m) + "\\\"";
}

/// temp solution while refactoring
//const std::string& type2Name_temp(const std::type_info& _info);



/// should return the CAST name of a feature, this function must be
/// specialized for each feature in \p FeatureProperties.h and \p
/// FeatureProperties.cpp
//template<class FeatureType>
//const std::string& 
//featureName() 
//{
 // throw std::runtime_error("featureName<FeatureType>: Not a feature... this function only defined for features. Did you forget to specialize featureName in Feature_Properties.h?");
//};


/// Creates a prettier name for features for presentation
/// purposes. It calls \p featureName() and uses the part of the
/// std::string of the name after the last ':' (unless the result of this
/// operation would be an empty std::string, in which case the full name is
/// returned). The function be specialized if you want a more specific
/// naming for your feature. (it calls \p trimFeatureName)

/// should return the properties of a feature. This function must be
/// specialized for each feature in \p FeatureProperties.h and \p
/// FeatureProperties.cpp.
// template<class FeatureType> 
// FeatureProperties featureProperties()  
// {
//   throw std::runtime_error("featureProperties<FeatureType>: Not a feature... this function only defined for features. Did you forget to specialize featureProperties in Feature_Properties.h?");
// };

/// default less operator for features, compares their addresses (the
/// operator< is not overloaded in this template since it may have
/// weird consequences)
// template<class FeatureType> 
// bool featureOperatorLess(const FeatureType& _f1, const FeatureType& _f2) {
//   return &_f1 < &_f2;
// }

// forward declaration
//class AnyFeature;

/// Tests if a feature is identical to another feature. The return
/// value is a tribool that should return true if the features are
/// identical, false if they are not, and possible if this cannot be
/// asserted. This function must be specialized for each feature in
/// FeatureProperties.h and FeatureProperties.cpp
/// \param _proxyFeature, must as the name suggests be member of a proxy
/// \param _unionFeature, must as the name suggests be member of an binding union
// template<class FeatureType1, class FeatureType2> 
// boost::logic::tribool featuresEquivalent(const FeatureType1& _proxyFeature, const FeatureType2& _unionFeature) {
//   throw(std::runtime_error("Comparison between " + featureName(_proxyFeature) + " and " + featureName(_unionFeature) 
// 		      + " not declared. Specialization for identical function for" + featureName(_proxyFeature) 
// 		      + " to "  + featureName(_unionFeature) + 
// 		      " should be declared in FeatureProperties.h and defined in FeatureProperties.cpp."));
// }

/// Tests if a feature is identical to another feature. The return
/// value is a boost::logic::tribool that should return true if the features are
/// identical, false if they are not, and possible if this cannot be
/// asserted. This function must be specialized for each feature in
/// FeatureProperties.h and FeatureProperties.cpp . It should also
/// call the
// template<class FeatureType> 
// boost::logic::tribool featuresEquivalent(const FeatureType& _proxyFeature, const AnyFeature& _unionFeature) {
//   throw(std::runtime_error("Comparison between " + featureName(_proxyFeature) + " and AnyFeature"
// 		      " not declared. Specialization for identical function for " + featureName(_proxyFeature) + 
// 		      " should be declared in FeatureProperties.h and defined in FeatureProperties.cpp."));
// }
// 

} // namespace Binding


#endif
