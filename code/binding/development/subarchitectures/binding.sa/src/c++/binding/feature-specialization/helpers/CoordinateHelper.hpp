#ifndef BINDING_COORDINATE_HELPER_H_
#define BINDING_COORDINATE_HELPER_H_

#include <ostream>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {


/// returnd indeterminate if the reference frames are not matching,
/// otherwise it returns true only if there is a exact match
class CoordinateComparator : public ReflexiveInternalComparator<BindingFeatures::Coordinate> 
{
public:
  CoordinateComparator() : 
    ReflexiveInternalComparator<BindingFeatures::Coordinate>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class CoordinateHelper : public FeatureHelper<BindingFeatures::Coordinate> {
public:
  CoordinateHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_COORDINATE_HELPER_H_
