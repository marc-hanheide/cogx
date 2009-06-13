#ifndef BINDING_FEATURE_ONTOLOGY_H_
#define BINDING_FEATURE_ONTOLOGY_H_

// #include <cast/core/CASTCore.hpp>
#include <cast/core/CASTUtils.hpp>
#include <map>
#include <set>
#include <string>
#include <cassert>
#include <BindingData.hpp>
#include "../feature-utils/FeaturePropertiesUtils.hpp"
#include "../feature-utils/ComparatorIndex.hpp"
#include "../utils/TypeMap.hpp"
//#include "feature-utils/FeatureHelper.hpp"

namespace Binding {
/// hard limit...
const unsigned int MAX_FEATURES = 64;


//fwd declarations
class AbstractInternalComparator;
//class BindingFeatureOntologyFactory;
class AbstractFeature;
class AbstractBinder;
class FeatureRegistrant;
class AbstractFeatureHelper;

/// manages binding features and the relates these data types to operators that can be applied to them
class BindingFeatureOntology
{


  friend class FeatureRegistrant;
public:
  /// singleton construct memfun
  static BindingFeatureOntology& construct() {
    static BindingFeatureOntology ontology;
    return ontology;
  }

protected:
  BindingFeatureOntology();
  virtual ~BindingFeatureOntology(){};

public:

  /// needs to be checked after refactoring..
  template<class T>
  void
  addFeatureObjectType(boost::shared_ptr<const AbstractFeatureHelper>& _featureHelper) {

    assert(ontologyState == FEATURE_REGISTRATION);
    std::string _dataType = cast::typeName<T>();
    std::string _featureType = cast::typeName<T>();
    allFeatureNames.insert(_featureType);
//    addObjectType<T>();

    std::cout << std::string("Initiating feature: ") << _featureType << std::endl;
    //assert(_featureType == Binding::featureName<T>());

    assertHelperType(*_featureHelper, typeid(T));

    assert(featureName.find(&typeid(T)) == featureName.end()); // only add each helper once

    featureName.insert(make_pair(&typeid(T), _featureType));
    unsigned int n = featureNumber.size();

    featureNumber.insert(make_pair(&typeid(T), n));
    assert(n + 1 == featureNumber.size());

    featureNameNumber.insert(make_pair(_featureType, n));
    assert(n + 1 == featureNameNumber.size());

    //featureHelpers.insert(make_pair(&typeid(T), _featureHelper));
    featureHelpers[n] = _featureHelper;

    assert(featureNumber.size() < MAX_FEATURES);

    updateFeatureProperties(_featureType, *_featureHelper);
  }

public:
  void updateFeatureProperties(const std::string& _featureType, const AbstractFeatureHelper&);
  void updateFeatureProperties(const std::string& _featureType, const FeatureProperties& _properties);

/*  struct FeatureStats {
    FeatureStats() :
      internalStat(*this),
      externalStat(*this) {}
    struct Stat {
      Stat(const FeatureStats& _stats) :
	matches(0),
	mismatches(0),
	indeterminates(0),
	comparisons(0),
	stats(_stats){}
      unsigned int matches;
      unsigned int mismatches;
      unsigned int indeterminates;
      unsigned int comparisons;
      const FeatureStats& stats;
      double quality() const {
	assert(comparisons == matches + mismatches+ indeterminates);
	return (static_cast<double>(mismatches) + 1.0) /
	  (static_cast<double>(comparisons) + 1.0) + static_cast<double>(comparisons) / 10.0;
      }
    };
    Stat internalStat;
    Stat externalStat;
    double quality() const {
      return internalStat.quality() * 20.0 + externalStat.quality();
    }

    bool operator<(const FeatureStats& _stats) {
      return this->quality() > _stats.quality();
    }
  };

  static cast::StringMap<FeatureStats>::map featureStats;
  static FeatureStats& featureStats(const std::string& _type) {return featureStats[_type];}
*/

  /// testing... should return a feature from wm
  const AbstractFeature& getFeature(AbstractBinder& _b, const std::string& _id);


public:
  /// a set of all feature type names, for debugging and assertions
  const std::set<std::string>& features() const {return features;}
  const std::set<std::string>& comparableFeatures() const {return comparableFeatures;}
  bool comparable(const std::string& _type) const
  {
    return comparableFeatures.find(_type) != comparableFeatures.end();
  }

  /// implementation based on old code... refactorig
  const std::set<std::string>& comparableInternally(const std::type_info&) const;
  const std::set<std::string>& comparableExternally(const std::type_info&) const;
  /// implementation based on old code... refactorig
  const std::set<std::string>& comparableInternally(const std::string&) const;
  const std::set<std::string>& comparableExternally(const std::string&) const;

  //static const std::map<std::string,Binding::FeatureProperties>& featurePropertyMap() {return featurePropertyMap;}
  const std::map<std::string,std::set<std::string> >& proxy2unionComparable() const {return proxy2unionComparable;}
  const std::map<std::string,std::set<std::string> >& union2proxyComparable() const {return union2proxyComparable;}


  typedef std::map<std::string,BindingData::ComparisonTrustSpecification> UnionToSpecMap;
  typedef std::map<std::string, UnionToSpecMap> TrustSpecMap;

  const TrustSpecMap& internalTrust() const {return internalTrust;}
  void registerComparatorCompetence(const BindingData::FeatureComparisonCompetence& _competence);
private:

  std::set<std::string> features;
  std::set<std::string> comparableFeatures;
  std::map<std::string,std::set<std::string> > proxy2unionComparable;
  std::map<std::string,std::set<std::string> > union2proxyComparable;

  std::map<std::string,Binding::FeatureProperties> featurePropertyMap;

  /// maps from proxy feature type std::string to union feature type std::string. Maps to the competence specification
  TrustSpecMap internalTrust;

  /// not copyable
  BindingFeatureOntology(const BindingFeatureOntology&);
  /// not assignable
  void operator=(const BindingFeatureOntology&);

  TypeMap<std::string>::map featureName;
  // a unique number per feature type
  TypeMap<unsigned int>::map featureNumber;
  // a unique number per feature type name
  cast::StringMap<unsigned int>::map featureNameNumber;


public: // for now
  boost::shared_ptr<const AbstractInternalComparator> internalComparators[MAX_FEATURES][MAX_FEATURES];
  /// one helper per feature type...
  //TypeMap<boost::shared_ptr<const AbstractFeatureHelper> >::map featureHelpers;
  boost::shared_ptr<const AbstractFeatureHelper> featureHelpers[MAX_FEATURES];

  void registerInternalComparator(const std::type_info& _proxyFeatureType,
				  const std::type_info& _unionFeatureType,
				  boost::shared_ptr<const AbstractInternalComparator>&);

private:
  std::set<std::string> comparableInternally[MAX_FEATURES];
  std::set<std::string> comparableExternally[MAX_FEATURES];

private:
  const AbstractFeatureHelper& featureHelper(unsigned int _featureNumber) const {
    return *featureHelpers[_featureNumber];
  }


  std::set<std::string> allFeatureNames;
public:
  /// returns the names of all features
  const std::set<std::string>& allFeatureNames() const {return allFeatureNames;}
  const std::string& featureName(const std::type_info&) const;
  unsigned int featureNumber(const std::type_info&) const;
  unsigned int featureNumber(const std::string&) const;
  unsigned int featureCount() const{return featureNumber.size();}
  const AbstractFeatureHelper& featureHelper(const std::type_info&) const;
  const AbstractFeatureHelper& featureHelper(const std::string& _name) const {return featureHelper(featureNumber(_name));}
  /// returns null if there is no appropriate comparator
  const AbstractInternalComparator* internalComparator(const std::type_info& proxyFeatureType, const std::type_info& unionFeatureType) const;
  //const AbstractFeatureHelper& featureHelper(const std::string&);
  /// for debugging purposes
  void openFeatureRegistration();
  /// for debugging purposes
  void freezeFeatures();
  /// for debugging purposes
  void freezeInternalComparators();
  /// for debugging purposes
  void constructorFinished();


private:
  /// for debugging purposes
  enum {
    PRE_CONSTRUCT, ///< before any constructor code is executed
    FEATURE_REGISTRATION, ///< while features and helpers are registered
    COMPARATOR_REGISTRATION, ///< while comparators are registered (feature registration not allowed)
    REGISTRATION_FROZEN, ///< no more registration of stuff (except external comparators)
    CONSTRUCTED, ///< Fully constructed while comparators are registered (feature or internal comparator registration not allowed)
  } ontologyState;

  /// used to track user mistakes
  void assertHelperType(const AbstractFeatureHelper&, const std::type_info& _info) const;

};



} // namespace Binding

#endif // BINDING_FEATURE_ONTOLOGY_H_
