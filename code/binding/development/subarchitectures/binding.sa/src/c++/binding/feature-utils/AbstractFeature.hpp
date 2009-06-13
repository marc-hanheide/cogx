#ifndef BINDING_ABSTRACT_FEATURE_H_
#define BINDING_ABSTRACT_FEATURE_H_

#include <string>
#include <map>
#include <set>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/logic/tribool.hpp>
#include <BindingData.hpp>
#include <BindingFeaturesCommon.hpp>
//#include "binding/ontology/BindingFeatureOntology.hpp"

//BOOST_TRIBOOL_THIRD_STATE(possible)

namespace Binding {
  class FeatureProperties;
  class AbstractMonitor;

/// abstract class that contains all the member functions for binding
/// features. These are implemented in the templated \c
/// Binding::Feature
class AbstractFeature {
public:
  enum FeatureDisplayMode {show_id,show_proxy_id,show_all,only_feature};
  /// overloaded in \p Feature
  std::string toString(FeatureDisplayMode _fs) const;
  /// overloaded in \p Feature
  std::string toDotLabel(FeatureDisplayMode _fs) const;
  /// overloaded in \p Feature, returns the typeinfo of the contained
  /// IDLFeature
  virtual const std::type_info& typeInfo() const = 0;
  /// calls appropriate function in \p BindingFeatureOntology
  const std::string& name() const;
  /// calls appropriate function in \p BindingFeatureOntology
  const std::string& shortName() const;
  /// calls appropriate function in \p BindingFeatureOntology
  const FeatureProperties& properties() const;
  /// calls appropriate function in \p BindingFeatureOntology
  bool operator<(const AbstractFeature& _f) const;
  /// based on operator<
  bool operator>(const AbstractFeature& _f) const {return _f < *this;}
  /// based on operator<
  bool operator==(const AbstractFeature& _f) const {return !(_f < *this) && !(*this < _f);}
  /// based on operator<
  bool operator!=(const AbstractFeature& _f) const {return (_f < *this) || (*this < _f);}
  /// calls appropriate function in \p BindingFeatureOntology
  bool comparable(const AbstractFeature& _f) const;  
  /// calls appropriate function in \p BindingFeatureOntology
  boost::logic::tribool compare(const AbstractFeature& _f) const;  

  /// true iff the feature is actually negated, which inverts the
  /// match (i.e. two objects that are 'not red' will actually match)
  virtual bool negated() const = 0;  
  virtual std::string immediateProxyID() const = 0;  
  virtual BindingFeaturesCommon::TruthValue truthValue() const = 0;

  friend std::ostream& operator<<(std::ostream&, const AbstractFeature&);
  
  /// used by \p CopyCatBindingMonitor
  // virtual BindingData::FeaturePointer addFeatureToCurrentProxy(AbstractMonitor& _monitor, BindingFeaturesCommon::TruthValue _truthValue) const = 0;

protected:
  AbstractFeature(const std::string& _id) 
    : id(_id) {};
  
public:
  virtual ~AbstractFeature() { };
  /// returns the id of the feature (i.e. it's adress on WM)
  const std::string& featureID() const {
    return id;
  }

private:
  std::string id;
  AbstractFeature() {};
};


struct AbstractFeaturePtrCmp {
  bool operator()(const boost::shared_ptr<AbstractFeature>& _f1, 
		  const boost::shared_ptr<AbstractFeature>& _f2) const {
    return *_f1 < *_f2;
  }
};

typedef std::set<boost::shared_ptr<AbstractFeature>, AbstractFeaturePtrCmp> OneTypeOfFeatures;
typedef std::map<std::string, OneTypeOfFeatures> FeatureSet;

  
struct AbstractFeaturePtrWithRepetitionsCmp {
  bool operator()(const boost::shared_ptr<AbstractFeature>& _f1, 
		  const boost::shared_ptr<AbstractFeature>& _f2) const {
    return _f1->featureID() <_f2->featureID();
  }
};


typedef std::set<boost::shared_ptr<AbstractFeature>, AbstractFeaturePtrWithRepetitionsCmp> OneTypeOfFeaturesWithRepetitions;
typedef std::map<std::string, OneTypeOfFeaturesWithRepetitions> FeatureSetWithRepetitions;


//struct OptimizedAbstractFeaturePtrCmp {
//  bool operator()(const boost::shared_ptr<AbstractFeature>& _f1, 
//		  const boost::shared_ptr<AbstractFeature>& _f2) const;
//};

//typedef std::set<boost::shared_ptr<AbstractFeature>, OptimizedAbstractFeaturePtrCmp> OptimizedOneTypeOfFeatures;
//typedef std::map<std::string, OptimizedOneTypeOfFeatures> OptimizedFeatureSet;

/// sorts the features according to quality, in terms of comparability
/// according to BindingFeatureOntology. Can be a bit dangerous if used
/// outside BindingScorer since BindingScorer is updating the quality
//struct OptimizedAbstractFeaturePtrWithRepetitionsCmp {
//  bool operator()(const boost::shared_ptr<AbstractFeature>& _f1, 
//		  const boost::shared_ptr<AbstractFeature>& _f2) const;
//};

//typedef std::set<boost::shared_ptr<AbstractFeature>, OptimizedAbstractFeaturePtrWithRepetitionsCmp> OptimizedOneTypeOfFeaturesWithRepetitions;
//typedef std::map<std::string, OptimizedOneTypeOfFeaturesWithRepetitions> OptimizedFeatureSetWithRepetitions;


/// inserts all features from \p _src into \p _dst. It would be easy to
/// optimize this a bit...
template<class FeatureSetT>
inline
void 
mergeFeatureSets(FeatureSetT& _dst, const FeatureSetT& _src)
{
  //FeatureSet::const_iterator dst = _dst.begin();
  for(FeatureSet::const_iterator src = _src.begin();
      src != _src.end(); 
      ++src) {
    _dst[src->first].insert(src->second.begin(),src->second.end());
  }
}

/// returns ParentFeature's truthValue
template<typename FeatureT>
BindingFeaturesCommon::TruthValue
truthValue(const FeatureT& _f) {
  return _f.parent.truthValue;
}

/// returns based on TruthValue in ParentFeature
template<typename FeatureT>
bool
negated(const FeatureT& _f) {
  return Binding::truthValue(_f) == BindingFeaturesCommon::NEGATIVE;
}

/// returns based on ParentFeature's immediateProxyID
template<typename FeatureT>
std::string
immediateProxyID(const FeatureT& _f) {
  return std::string(_f.parent.immediateProxyID);
}



} // namespace Binding

#endif
