#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "FeatureComparatorExample.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const std::string &_id) {
    return new Binding::FeatureComparatorExample::FeatureComparatorExample(_id);
  }
}

namespace Binding {
using namespace std;
using namespace boost;
using namespace cast;

FeatureComparatorExample::FeatureComparatorExample(const std::string& _id)
  : 
    WorkingMemoryAttachedComponent(_id),
    AbstractFeatureComparator(_id),
    featureCacheConcept(*this) { }


void
FeatureComparatorExample::startComparator()
{
  BindingData::ComparisonTrustSpecification comparisonTrustSpecification;
  comparisonTrustSpecification.trustInternalFalse = BindingData::DONT_USE;
  comparisonTrustSpecification.trustInternalIndeterminate = BindingData::DONT_USE;
  comparisonTrustSpecification.trustInternalTrue = BindingData::TRUST_COMPLETELY;
  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  addFeatureComparisonFilter(ont.featureName(typeid(BindingFeatures::Concept)), 
			     comparisonTrustSpecification);
}

boost::logic::tribool 
FeatureComparatorExample::executeComparison()
{
  
  const BindingFeatures::Concept& 
    proxyConcept(featureCacheConcept[string(currentComparison().proxyFeature.address)]);
  const BindingFeatures::Concept& 
    unionConcept(featureCacheConcept[string(currentComparison().unionFeature.address)]);
  log("COMPARING " + string(proxyConcept.concept) + " vs " + string(unionConcept.concept));

  if(negated(proxyConcept) == negated(unionConcept))
    return string(proxyConcept.concept) == string(unionConcept.concept);
  else
    return string(proxyConcept.concept) != string(unionConcept.concept);
}

} // namespace Binding
