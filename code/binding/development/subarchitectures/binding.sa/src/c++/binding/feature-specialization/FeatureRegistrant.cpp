#include "FeatureRegistrant.hpp"
#include "binding/feature-utils/ComparatorIndex.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/feature-specialization/helpers/IncludeAllFeatureHelpers.hpp"
#include <binding/idl/BindingFeatures.hh>
#include <binding/utils/BindingUtils.hpp>
#include <boost/shared_ptr.hpp>
#include <cast/core/CASTUtils.hpp>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach

namespace Binding {
  using namespace boost;
  using namespace std;

void 
FeatureRegistrant::registerInternalComparator(BindingFeatureOntology& _ontology, 
					      shared_ptr<const AbstractInternalComparator> _comp)
{
  _comp->registerWithOntology(_ontology, _comp);
}

void
FeatureRegistrant::registerFeatures(BindingFeatureOntology& _ontology) {
  _ontology.openFeatureRegistration();
  
  shared_ptr<const AbstractFeatureHelper> helper(new ConceptHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Concept>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new RelationLabelHelper);
  _ontology.addFeatureObjectType<BindingFeatures::RelationLabel>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new NameHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Name>(helper);
  // deprecated  UGLY_addFeatureObjectType<BindingFeatures::Reachable>(_ontology);
  // deprecatedUGLY_addFeatureObjectType<BindingFeatures::Visible>(_ontology);
  helper = shared_ptr<const AbstractFeatureHelper>(new DebugStringHelper);
  _ontology.addFeatureObjectType<BindingFeatures::DebugString>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new ShapeHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Shape>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new SizeHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Size>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new ColourHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Colour>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new LocationHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Location>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new ExistingProxyIDHelper);
  _ontology.addFeatureObjectType<BindingFeatures::ExistingProxyID>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new ThisProxyIDHelper);
  _ontology.addFeatureObjectType<BindingFeatures::ThisProxyID>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new SourceIDHelper);
  _ontology.addFeatureObjectType<BindingFeatures::SourceID>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new OtherSourceIDHelper);
  _ontology.addFeatureObjectType<BindingFeatures::OtherSourceID>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new SourceDataHelper);
  _ontology.addFeatureObjectType<BindingFeatures::SourceData>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new CreationTimeHelper);
  _ontology.addFeatureObjectType<BindingFeatures::CreationTime>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new GroupHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Group>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new SingularHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Singular>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new SalienceHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Salience>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new TemporalFrameHelper);
  _ontology.addFeatureObjectType<BindingFeatures::TemporalFrame>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new AreaIDHelper);
  _ontology.addFeatureObjectType<BindingFeatures::AreaID>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new AspectualStateHelper);
  _ontology.addFeatureObjectType<BindingFeatures::AspectualState>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new PersonIDHelper);
  _ontology.addFeatureObjectType<BindingFeatures::PersonID>(helper);
  
#ifdef EXPERIMENTAL_BINDING_FEATURES
  helper = shared_ptr<const AbstractFeatureHelper>(new CoordinateHelper);
  _ontology.addFeatureObjectType<BindingFeatures::Coordinate>(helper);
  helper = shared_ptr<const AbstractFeatureHelper>(new LineOfSightHelper);
  _ontology.addFeatureObjectType<BindingFeatures::LineOfSight>(helper);
#endif //EXPERIMENTAL_BINDING_FEATURES
  
  // deprecated  UGLY_addFeatureObjectType<BindingFeatures::Proposition>(_ontology);
  
  /* deprecated
     helper = shared_ptr<const AbstractFeatureHelper>(new VisualColourHelper);
     _ontology.addFeatureObjectType<BindingFeatures::VisualColour>(helper);
     helper = shared_ptr<const AbstractFeatureHelper>(new VisualShapeHelper);
     _ontology.addFeatureObjectType<BindingFeatures::VisualShape>(helper);
     helper = shared_ptr<const AbstractFeatureHelper>(new VisualSizeHelper);
     _ontology.addFeatureObjectType<BindingFeatures::VisualSize>(helper);
     helper = shared_ptr<const AbstractFeatureHelper>(new VisualGenericHelper);
     _ontology.addFeatureObjectType<BindingFeatures::VisualGeneric>(helper);
  */

  _ontology.freezeFeatures(); // to allow adding comparators
  

  shared_ptr<const AbstractInternalComparator> comparator;
  comparator = shared_ptr<const AbstractInternalComparator>(new ConceptComparator);  
  registerInternalComparator(_ontology,comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new NameComparator);  
  registerInternalComparator(_ontology,comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new RelationLabelComparator);  
  registerInternalComparator(_ontology,comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new ColourComparator);  
  registerInternalComparator(_ontology,comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new ShapeComparator);  
  registerInternalComparator(_ontology,comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new SizeComparator);  
  registerInternalComparator(_ontology,comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new OtherSourceIDvsSourceIDComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new SourceDataComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new GroupComparator);
  registerInternalComparator(_ontology,comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new SingularComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new ExistingProxyIDvsThisProxyIDComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new LocationComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new TemporalFrameComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new SalienceVsSalienceComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new SalienceVsCreationTimeComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new AreaIDComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new AspectualStateComparator);
  registerInternalComparator(_ontology, comparator);
  comparator = shared_ptr<const AbstractInternalComparator>(new PersonIDComparator);
  registerInternalComparator(_ontology, comparator);
#ifdef EXPERIMENTAL_BINDING_FEATURES
  comparator = shared_ptr<const AbstractInternalComparator>(new CoordinateComparator);
  registerInternalComparator(_ontology, comparator);
#endif //EXPERIMENTAL_BINDING_FEATURES
  
  /* deprecated
     comparator = shared_ptr<const AbstractInternalComparator>(new VisualColourComparator);
     registerInternalComparator(_ontology, comparator);
     comparator = shared_ptr<const AbstractInternalComparator>(new VisualShapeComparator);
     registerInternalComparator(_ontology,comparator);
     comparator = shared_ptr<const AbstractInternalComparator>(new VisualSizeComparator);
     registerInternalComparator(_ontology,comparator);
     comparator = shared_ptr<const AbstractInternalComparator>(new VisualGenericComparator);
     registerInternalComparator(_ontology, comparator);
  */
  _ontology.freezeInternalComparators();
    
  foreach(const string& featureID, _ontology.features()) {
    const set<string>& internally_comparable_set(_ontology.comparableInternally(featureID));
    cout << featureID << " is comparable with ";
    print_set(cout, internally_comparable_set) << endl;
  }
  cout << "All comparable features: ";
  print_set(cout, _ontology.comparableFeatures()) << endl;
}


} //namespace Binding 
