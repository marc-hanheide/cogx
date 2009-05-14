#ifndef BINDING_VISION_HELPERS_H_
#define BINDING_VISION_HELPERS_H_

#include <ostream>
#include <idl/BindingFeaturesCommon.hh>
#include <idl/BindingFeatures.hh>
#include "feature-utils/FeatureHelper.hpp"
#include "feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class VisualColourComparator : public ReflexiveInternalComparator<BindingFeatures::VisualColour> 
{
public:
  VisualColourComparator() : 
    ReflexiveInternalComparator<BindingFeatures::VisualColour>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class VisualColourHelper : public FeatureHelper<BindingFeatures::VisualColour> {
public:
  VisualColourHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};



class VisualShapeComparator : public ReflexiveInternalComparator<BindingFeatures::VisualShape> 
{
public:
  VisualShapeComparator() : 
    ReflexiveInternalComparator<BindingFeatures::VisualShape>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class VisualShapeHelper : public FeatureHelper<BindingFeatures::VisualShape> {
public:
  VisualShapeHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};



class VisualSizeComparator : public ReflexiveInternalComparator<BindingFeatures::VisualSize> 
{
public:
  VisualSizeComparator() : 
    ReflexiveInternalComparator<BindingFeatures::VisualSize>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class VisualSizeHelper : public FeatureHelper<BindingFeatures::VisualSize> {
public:
  VisualSizeHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};




class VisualGenericComparator : public ReflexiveInternalComparator<BindingFeatures::VisualGeneric> 
{
public:
  VisualGenericComparator() : 
    ReflexiveInternalComparator<BindingFeatures::VisualGeneric>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class VisualGenericHelper : public FeatureHelper<BindingFeatures::VisualGeneric> {
public:
  VisualGenericHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};


} // namespace Binding

#endif // BINDING_VISION_HELPERS_H_
