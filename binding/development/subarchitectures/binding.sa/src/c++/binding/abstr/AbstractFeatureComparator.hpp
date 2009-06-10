#ifndef BINDING_ABSTRACT_FEATURE_COMPARATOR_H_
#define BINDING_ABSTRACT_FEATURE_COMPARATOR_H_

#include "binding/idl/BindingData.hh"
#include "cast/core/CASTDataCache.hpp"
#include "binding/BindingException.hpp"

#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>

namespace Binding {

class AbstractFeatureComparator: 
    public cast::PrivilegedManagedProcess {
      
public:
  /// Initiates the filter which receives FeatureComparisonTasks. No
  /// need to overload this. It will call your overloaded
  /// startComparator instead (tis way you can't possibly forget
  /// calling AbstractFeatureComparator::start() in any overloaded
  /// start-method).
  void start();

  /// If you overload configure, don't forget to call \p
  /// AbstractFeatureComparator::configure(...) too! (Maybe this
  /// should be made safer, as with start()...)
  virtual void configure(std::map<std::string,std::string>& _config);
  
protected:

  /// Calls startComparator(). If you overide this class and uses
  /// runComponent, make sure you call
  /// AbstractFeatureComparator::runComponent() at the start of your
  /// runComponent method
  virtual void runComponent();

  /// adds the pair of features to the ones that will should be
  /// compared by the comparator. The types can be different (if the
  /// comparator is nonreflexive), but are typically not. Use this
  /// function in the startComparator method. The method also
  /// registers the comparison competence on each binding WM set with
  /// the --bsa flag.
  void addFeatureComparisonFilter(const std::string& _proxyFeatureType, 
				  const std::string& _unionFeatureType,
				  const BindingData::ComparisonTrustSpecification& _comparisonTrustSpecification);

  /// calls addFeatureComparisonFilter(_featureType,_featureType)...
  void addFeatureComparisonFilter(const std::string& _featureType,
				  const BindingData::ComparisonTrustSpecification& _comparisonTrustSpecification) {
    addFeatureComparisonFilter(_featureType, _featureType, _comparisonTrustSpecification);
  }

  /// calls addFeatureComparisonFilter(_featureType1,_featureType2)...
  /// and addFeatureComparisonFilter(_featureType2,_featureType1)...
  void addBidirectionalFeatureComparisonFilter(const std::string& _featureType1, 
					       const std::string& _featureType2,
					       const BindingData::ComparisonTrustSpecification& _comparisonTrustSpecification) {
    addFeatureComparisonFilter(_featureType1, _featureType2, _comparisonTrustSpecification);
    addFeatureComparisonFilter(_featureType2, _featureType1, _comparisonTrustSpecification);
  }
  
  /// overload this with your code. This is supposed to return true,
  /// false or indeterminate based on whatever algorithm you are using
  /// to do this. You will need to load the feature data yourself (you
  /// should add a \p cast::CASTDataCache member for each relevant
  /// feature since many features will be loaded many times). Access
  /// the featureComparison via current
  virtual boost::logic::tribool executeComparison() = 0;
  
  /// is called from this->start() and must be overloaded with the
  /// relevant calls to \p addFeatureComparisonFilter
  virtual void startComparator() = 0;
  
  AbstractFeatureComparator(const std::string &_id);
  virtual ~AbstractFeatureComparator() { };

  
protected:
  struct Comparison {
    Comparison() :
      featureComparison(boost::shared_ptr<const BindingData::FeatureComparison>()) { }
    boost::shared_ptr<const BindingData::FeatureComparison> featureComparison;
    std::string id;
    boost::logic::tribool originalValue;
    boost::logic::tribool newValue;
    /// describes in what way the comparator is competent for the current comparison
    BindingData::ComparisonTrustSpecification m_comparisonTrustSpecification;
  };
  
  /// use this to get the feature comparison that you're supposed to
  /// process
  const BindingData::FeatureComparison& currentComparison() const throw(Binding::BindingException) {
    if(m_currentComparison.featureComparison == NULL) {
      throw Binding::BindingException("No current comparison loaded error. Did you check if currentComparisonIsMyTask() returns true before accessing the comparison with currentComparison()?");
    }
    return *m_currentComparison.featureComparison;
  }
  
  /// returns true if the current comparison task is relevant for this
  /// comparator (according to \p m_filter)
  bool currentComparisonIsMyTask() const;
  
private:
  Comparison m_currentComparison;

  typedef std::map<std::string,BindingData::ComparisonTrustSpecification> UnionToSpecMap;
  typedef std::map<std::string, UnionToSpecMap> FilterMap;
  /// maps from proxy feature type string to union feature type string. Maps to the competence specification
  FilterMap m_filter;
  
  /// is called as soon as a comparison task is added to wm. This
  /// loads the feature comparison struct. And if it this supposed to
  /// be evaluated by this comparatr, then the task itself is deleted
  /// from WM, the features are evaluated (i.e., \p executeComparison
  /// is called) and if the result differs from the stored one, it
  /// will overwrite the old score.
  void _processFeatureComparisonTask(const cast::cdl::WorkingMemoryChange& _wmc);

protected:
  /// the IDs of all bindingSAs set with the --bsa flag
  std::set<std::string> m_bindingSA;
  
public:
  /// returns the IDs of bindingSAs set with the --bsa flag
  const std::set<std::string>& bindingSA() const {return m_bindingSA;}
};

} // namespace Binding

#endif // BINDING_ABSTRACT_FEATURE_COMPARATOR_H_
