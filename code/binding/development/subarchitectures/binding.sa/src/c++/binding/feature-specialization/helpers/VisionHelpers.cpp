#include "VisionHelpers.hpp"

namespace Binding {
using namespace std;

boost::tribool 
VisualColourComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualColour& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualColour& unionFeature(getIDLFeature(_unionFeature));    
//  cout << "VisualColourComparator::compare: I am called with " << proxyFeature.m_visualColour << " vs. " << unionFeature.m_visualColour << endl;

  return proxyFeature.m_visualColour == unionFeature.m_visualColour;
}

VisualColourHelper::VisualColourHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant   = false;
  prop.m_isSimplex     = true;
  prop.m_isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualColourHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).m_visualColour;
  return _out;
}

bool 
VisualColourHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).m_visualColour < extract(_feat2).m_visualColour;
}



boost::tribool 
VisualShapeComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualShape& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualShape& unionFeature(getIDLFeature(_unionFeature));    
  return proxyFeature.m_visualShape == unionFeature.m_visualShape;
}

VisualShapeHelper::VisualShapeHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant   = false;
  prop.m_isSimplex     = true;
  prop.m_isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualShapeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).m_visualShape;
  return _out;
}

bool 
VisualShapeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).m_visualShape < extract(_feat2).m_visualShape;
}
  

boost::tribool 
VisualSizeComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualSize& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualSize& unionFeature(getIDLFeature(_unionFeature));    
//  cout << "VisualSizeComparator::compare: I am called with " << proxyFeature.m_visualSize << " vs. " << unionFeature.m_visualSize << endl;

  return proxyFeature.m_visualSize == unionFeature.m_visualSize;
}

VisualSizeHelper::VisualSizeHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant   = false;
  prop.m_isSimplex     = true;
  prop.m_isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualSizeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).m_visualSize;
  return _out;
}

bool 
VisualSizeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).m_visualSize < extract(_feat2).m_visualSize;
}


boost::tribool 
VisualGenericComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::VisualGeneric& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::VisualGeneric& unionFeature(getIDLFeature(_unionFeature));    
//  cout << "VisualGenericComparator::compare: I am called with " << proxyFeature.m_visualGeneric << " vs. " << unionFeature.m_visualGeneric << endl;

  return proxyFeature.m_visualGeneric == unionFeature.m_visualGeneric;
}

VisualGenericHelper::VisualGenericHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant   = false;
  prop.m_isSimplex     = true;
  prop.m_isDemanded    = false;
  setProperties(prop);
}

ostream& 
VisualGenericHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "# " << extract(_feat).m_visualGeneric;
  return _out;
}

bool 
VisualGenericHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).m_visualGeneric < extract(_feat2).m_visualGeneric;
}


} // namespace Binding
