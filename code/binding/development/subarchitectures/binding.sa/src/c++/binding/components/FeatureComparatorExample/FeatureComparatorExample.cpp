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
    m_featureCacheConcept(*this) { }


void
FeatureComparatorExample::startComparator()
{
  BindingData::ComparisonTrustSpecification comparisonTrustSpecification;
  comparisonTrustSpecification.m_trustInternalFalse = BindingData::DONT_USE;
  comparisonTrustSpecification.m_trustInternalIndeterminate = BindingData::DONT_USE;
  comparisonTrustSpecification.m_trustInternalTrue = BindingData::TRUST_COMPLETELY;
  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  addFeatureComparisonFilter(ont.featureName(typeid(BindingFeatures::Concept)), 
			     comparisonTrustSpecification);
}

boost::logic::tribool 
FeatureComparatorExample::executeComparison()
{
  
  const BindingFeatures::Concept& 
    proxyConcept(m_featureCacheConcept[string(currentComparison().m_proxyFeature.m_address)]);
  const BindingFeatures::Concept& 
    unionConcept(m_featureCacheConcept[string(currentComparison().m_unionFeature.m_address)]);
  log("COMPARING " + string(proxyConcept.m_concept) + " vs " + string(unionConcept.m_concept));

  if(negated(proxyConcept) == negated(unionConcept))
    return string(proxyConcept.m_concept) == string(unionConcept.m_concept);
  else
    return string(proxyConcept.m_concept) != string(unionConcept.m_concept);
}

} // namespace Binding
