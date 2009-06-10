#ifndef BINDING_FEATURE_ONTOLOGY_H_
#define BINDING_FEATURE_ONTOLOGY_H_

#include <cast/core/CASTCore.hpp>
#include <cast/core/CASTUtils.hpp>
#include <map>
#include <set>
#include <string>
#include <cassert>
#include <binding/idl/BindingData.hh>
#include "binding/feature-utils/FeaturePropertiesUtils.hpp"
#include "binding/feature-utils/ComparatorIndex.hpp"
#include "binding/utils/TypeMap.hpp"
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

    assert(m_ontologyState == FEATURE_REGISTRATION);
    std::string _dataType = cast::typeName<T>();
    std::string _featureType = cast::typeName<T>();
    m_allFeatureNames.insert(_featureType);
//    addObjectType<T>();

    std::cout << std::string("Initiating feature: ") << _featureType << std::endl; 
    //assert(_featureType == Binding::featureName<T>());    
        
    assertHelperType(*_featureHelper, typeid(T));
    
    assert(m_featureName.find(&typeid(T)) == m_featureName.end()); // only add each helper once 
    
    m_featureName.insert(make_pair(&typeid(T), _featureType));
    unsigned int n = m_featureNumber.size();
    
    m_featureNumber.insert(make_pair(&typeid(T), n));
    assert(n + 1 == m_featureNumber.size());
    
    m_featureNameNumber.insert(make_pair(_featureType, n));
    assert(n + 1 == m_featureNameNumber.size());
    
    //m_featureHelpers.insert(make_pair(&typeid(T), _featureHelper));
    m_featureHelpers[n] = _featureHelper;
    
    assert(m_featureNumber.size() < MAX_FEATURES);
    
    updateFeatureProperties(_featureType, *_featureHelper);
  }

public:
  void updateFeatureProperties(const std::string& _featureType, const AbstractFeatureHelper&); 
  void updateFeatureProperties(const std::string& _featureType, const FeatureProperties& _properties); 

/*  struct FeatureStats {
    FeatureStats() :
      m_internalStat(*this),
      m_externalStat(*this) {}
    struct Stat {
      Stat(const FeatureStats& _stats) :
	m_matches(0),
	m_mismatches(0),
	m_indeterminates(0),
	m_comparisons(0),
	m_stats(_stats){}
      unsigned int m_matches;
      unsigned int m_mismatches;
      unsigned int m_indeterminates;
      unsigned int m_comparisons;
      const FeatureStats& m_stats;      
      double quality() const {
	assert(m_comparisons == m_matches + m_mismatches+ m_indeterminates);
	return (static_cast<double>(m_mismatches) + 1.0) /
	  (static_cast<double>(m_comparisons) + 1.0) + static_cast<double>(m_comparisons) / 10.0;
      }
    };
    Stat m_internalStat;
    Stat m_externalStat;
    double quality() const {
      return m_internalStat.quality() * 20.0 + m_externalStat.quality();
    }
    
    bool operator<(const FeatureStats& _stats) {
      return this->quality() > _stats.quality();
    }
  };

  static cast::StringMap<FeatureStats>::map m_featureStats;
  static FeatureStats& featureStats(const std::string& _type) {return m_featureStats[_type];}
*/

  /// testing... should return a feature from wm
  const AbstractFeature& getFeature(AbstractBinder& _b, const std::string& _id);
  
  
public:
  /// a set of all feature type names, for debugging and assertions
  const std::set<std::string>& features() const {return m_features;}
  const std::set<std::string>& comparableFeatures() const {return m_comparableFeatures;}
  bool comparable(const std::string& _type) const
  {
    return m_comparableFeatures.find(_type) != m_comparableFeatures.end();
  }

  /// implementation based on old code... refactorig
  const std::set<std::string>& comparableInternally(const std::type_info&) const;
  const std::set<std::string>& comparableExternally(const std::type_info&) const;
  /// implementation based on old code... refactorig
  const std::set<std::string>& comparableInternally(const std::string&) const;
  const std::set<std::string>& comparableExternally(const std::string&) const;
  
  //static const std::map<std::string,Binding::FeatureProperties>& featurePropertyMap() {return m_featurePropertyMap;}
  const std::map<std::string,std::set<std::string> >& proxy2unionComparable() const {return m_proxy2unionComparable;}
  const std::map<std::string,std::set<std::string> >& union2proxyComparable() const {return m_union2proxyComparable;}
  

  typedef std::map<std::string,BindingData::ComparisonTrustSpecification> UnionToSpecMap;
  typedef std::map<std::string, UnionToSpecMap> TrustSpecMap;

  const TrustSpecMap& internalTrust() const {return m_internalTrust;}
  void registerComparatorCompetence(const BindingData::FeatureComparisonCompetence& _competence);
private:  
  
  std::set<std::string> m_features;
  std::set<std::string> m_comparableFeatures;
  std::map<std::string,std::set<std::string> > m_proxy2unionComparable;
  std::map<std::string,std::set<std::string> > m_union2proxyComparable;
  
  std::map<std::string,Binding::FeatureProperties> m_featurePropertyMap;
  
  /// maps from proxy feature type std::string to union feature type std::string. Maps to the competence specification
  TrustSpecMap m_internalTrust;

  /// not copyable
  BindingFeatureOntology(const BindingFeatureOntology&);
  /// not assignable
  void operator=(const BindingFeatureOntology&);
  
  TypeMap<std::string>::map m_featureName;
  // a unique number per feature type
  TypeMap<unsigned int>::map m_featureNumber;
  // a unique number per feature type name
  cast::StringMap<unsigned int>::map m_featureNameNumber;


public: // for now
  boost::shared_ptr<const AbstractInternalComparator> m_internalComparators[MAX_FEATURES][MAX_FEATURES];
  /// one helper per feature type...
  //TypeMap<boost::shared_ptr<const AbstractFeatureHelper> >::map m_featureHelpers;
  boost::shared_ptr<const AbstractFeatureHelper> m_featureHelpers[MAX_FEATURES];

  void registerInternalComparator(const std::type_info& _proxyFeatureType, 
				  const std::type_info& _unionFeatureType,
				  boost::shared_ptr<const AbstractInternalComparator>&);

private:
  std::set<std::string> m_comparableInternally[MAX_FEATURES];
  std::set<std::string> m_comparableExternally[MAX_FEATURES];

private:
  const AbstractFeatureHelper& featureHelper(unsigned int _featureNumber) const {
    return *m_featureHelpers[_featureNumber];
  }


  std::set<std::string> m_allFeatureNames;
public:
  /// returns the names of all features
  const std::set<std::string>& allFeatureNames() const {return m_allFeatureNames;}
  const std::string& featureName(const std::type_info&) const;
  unsigned int featureNumber(const std::type_info&) const;
  unsigned int featureNumber(const std::string&) const;
  unsigned int featureCount() const{return m_featureNumber.size();}
  const AbstractFeatureHelper& featureHelper(const std::type_info&) const;
  const AbstractFeatureHelper& featureHelper(const std::string& _name) const {return featureHelper(featureNumber(_name));}
  /// returns null if there is no appropriate comparator 
  const AbstractInternalComparator* internalComparator(const std::type_info& proxyFeatureType, const std::type_info& unionFeatureType) const;
  //const AbstractFeatureHelper& m_featureHelper(const std::string&);
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
  } m_ontologyState;

  /// used to track user mistakes
  void assertHelperType(const AbstractFeatureHelper&, const std::type_info& _info) const;

};



} // namespace Binding

#endif // BINDING_FEATURE_ONTOLOGY_H_
