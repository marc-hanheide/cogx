#ifndef BINDING_FEATURE_COMPARATOR_EXAMPLE_H_
#define BINDING_FEATURE_COMPARATOR_EXAMPLE_H_

#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>
#include "binding/abstr/AbstractFeatureComparator.hpp"

namespace Binding {

/// an example of a feature comparator that compares concept features
class FeatureComparatorExample : public AbstractFeatureComparator {
public:

  FeatureComparatorExample(const std::string& _id);
  virtual ~FeatureComparatorExample() {}

  virtual void runComponent() {};  
  
  virtual void startComparator();
  virtual boost::logic::tribool executeComparison();

protected:
  virtual void taskAdopted(const std::string &_taskID) {};
  virtual void taskRejected(const std::string &_taskID) {};
  virtual void configure(std::map<std::string,std::string>& _config) 
  {
    AbstractFeatureComparator::configure(_config);
  } 

private:
  cast::CASTDataCache<BindingFeatures::Concept> featureCacheConcept;
};

} // namespace Binding 

#endif // BINDING_FEATURE_COMPARATOR_EXAMPLE_H_
