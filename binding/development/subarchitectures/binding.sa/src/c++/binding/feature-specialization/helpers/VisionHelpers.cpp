#include "VisionHelpers.hpp"

namespace Binding {
using namespace std;

boost::tribool 
VisualColourComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualColour& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualColour& unionFeature(getIDLFeature(_unionFeature));    
//  cout << "VisualColourComparator::compare: I am called with " << proxyFeature.visualColour << " vs. " << unionFeature.visualColour << endl;

  return proxyFeature.visualColour == unionFeature.visualColour;
}

VisualColourHelper::VisualColourHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant   = false;
  prop.isSimplex     = true;
  prop.isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualColourHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).visualColour;
  return _out;
}

bool 
VisualColourHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).visualColour < extract(_feat2).visualColour;
}



boost::tribool 
VisualShapeComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualShape& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualShape& unionFeature(getIDLFeature(_unionFeature));    
  return proxyFeature.visualShape == unionFeature.visualShape;
}

VisualShapeHelper::VisualShapeHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant   = false;
  prop.isSimplex     = true;
  prop.isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualShapeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).visualShape;
  return _out;
}

bool 
VisualShapeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).visualShape < extract(_feat2).visualShape;
}
  

boost::tribool 
VisualSizeComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualSize& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualSize& unionFeature(getIDLFeature(_unionFeature));    
//  cout << "VisualSizeComparator::compare: I am called with " << proxyFeature.visualSize << " vs. " << unionFeature.visualSize << endl;

  return proxyFeature.visualSize == unionFeature.visualSize;
}

VisualSizeHelper::VisualSizeHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant   = false;
  prop.isSimplex     = true;
  prop.isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualSizeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).visualSize;
  return _out;
}

bool 
VisualSizeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).visualSize < extract(_feat2).visualSize;
}


boost::tribool 
VisualGenericComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualGeneric& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualGeneric& unionFeature(getIDLFeature(_unionFeature));    
//  cout << "VisualGenericComparator::compare: I am called with " << proxyFeature.visualGeneric << " vs. " << unionFeature.visualGeneric << endl;

  return proxyFeature.visualGeneric == unionFeature.visualGeneric;
}

VisualGenericHelper::VisualGenericHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant   = false;
  prop.isSimplex     = true;
  prop.isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualGenericHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).visualGeneric;
  return _out;
}

bool 
VisualGenericHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).visualGeneric < extract(_feat2).visualGeneric;
}


} // namespace Binding
