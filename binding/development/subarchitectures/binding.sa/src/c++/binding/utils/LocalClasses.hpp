#ifndef BINDING_LOCAL_CLASSES_H_
#define BINDING_LOCAL_CLASSES_H_

#include "BindingData.hpp"
//#include "binding/feature-utils/AbstractFeature.hpp"
#include "cast/core/CASTDataCache.hpp"
#include "../BindingException.hpp"
#include "../feature-utils/FeatureExtractor.hpp"
#include <set>

namespace BindingFeatures {
  // forward decl
  class Singular;
  class Group;
  namespace details {
    class GroupDetails;
  }
}

namespace Binding {


static const std::string& ambiguityIDPostFix() {
  static std::string amb(":ambiguity");
  return amb;
}

struct proxyPortLess {
  bool operator()(const BindingData::ProxyPort& _port1,const BindingData::ProxyPort& _port2) const;
};

typedef std::map<std::string,std::set<BindingData::ProxyPort, proxyPortLess> > PortMap;

/// used internally...
extern PortMap toPortMap(const BindingData::ProxyPorts& _ports);

// forward decl
class AbstractBindingWMRepresenter;

class LocalBindingData {
  /// contains all features
  mutable std::auto_ptr<FeatureSet> featureSet;
  /// contains only comparable features
  mutable std::auto_ptr<FeatureSet> comparableFeatureSet;
  /// contains all features
  mutable std::auto_ptr<FeatureSetWithRepetitions> featureSetWithRepetitions;
  /// contains only comparable features
  mutable std::auto_ptr<FeatureSetWithRepetitions> comparableFeatureSetWithRepetitions;
  /// contains all features (sorted)
  //  mutable std::auto_ptr<OptimizedFeatureSet> optimizedComparableFeatureSet;
  /// contains only comparable features (sorted)
  //  mutable std::auto_ptr<OptimizedFeatureSetWithRepetitions> optimizedComparableFeatureSetWithRepetitions;

  /// used for processing the featuresets primarily
  mutable AbstractBindingWMRepresenter& bindingWMRepresenter;
  /// union or proxy features
  const BindingData::FeaturePointers& features;
  /// translates a feature set from the features stored on CAST WM
  FeatureSet _toFeatureSet(const BindingData::FeaturePointers& features) const;
  FeatureSetWithRepetitions _toFeatureSetWithRepetitions(const BindingData::FeaturePointers& features) const;

  /// only includes features that can be compared with some other feature
  FeatureSet _toComparableFeatureSet(const BindingData::FeaturePointers& features) const;
  FeatureSetWithRepetitions _toComparableFeatureSetWithRepetitions(const BindingData::FeaturePointers& features) const;

  /// only includes features that can be compared with some other feature (sorted)
//  OptimizedFeatureSet _toOptimizedComparableFeatureSet(const BindingData::FeaturePointers& features) const;
//  OptimizedFeatureSetWithRepetitions _toOptimizedComparableFeatureSetWithRepetitions(const BindingData::FeaturePointers& features) const;

  mutable std::auto_ptr<const PortMap> outPortMap;

  const std::string id;
  const std::string subarchitecture_id;
  const unsigned int version;
protected:

  LocalBindingData(AbstractBindingWMRepresenter& _bindingWMRepresenter,
		   const BindingData::FeaturePointers& _features,
		   const std::string& _id,
		   const std::string& _subarchitecture_id,
		   unsigned int _version);

  virtual const BindingData::ProxyPorts& inPorts() const = 0;
  virtual const BindingData::ProxyPorts& rawOutPorts() const = 0;

  virtual ~LocalBindingData(){}
public:
  /// translates the unordered set of features into a \p FeatureSet (if a feature has several values, it will occur several times)
  /// \remark see documentation of \p FeatureSet to get an idea of how to access the features
  /// \remark see \p FeatureExtractor and \p extractFeature for convenient functions
  /// \remark this, and the memfuns similar to it are lazy-evaluating and caching. so it only costs to call it the first time, and nothing to not call it at all
  const FeatureSet& featureSet() const;
  /// translates the unordered set of features into a \p FeatureSet (but skips all features that are not comparable)
  const FeatureSet& comparableFeatureSet() const;
  /// same as featureSet() but with repetitions (e.g. Colour red several times)
  /// \remark primarily for internal use in the binder
  const FeatureSetWithRepetitions& featureSetWithRepetitions() const;
  /// same as \p comparableFeatureSet() but with repetitions (e.g. Colour red several times)
  /// \remark primarily for internal use in the binder
  const FeatureSetWithRepetitions& comparableFeatureSetWithRepetitions() const;
  /// returns true if there is at least one feature with a matching typename
  /// \remark implemented via \p featureSet()
  bool hasFeature(const std::string& _typeName) const;
  /// convenience function for \p hasFeature(const std::string&)
  bool hasFeature(const std::type_info& _type) const;
  /// also convenient
  template<typename FeatureT>
  bool hasFeature() const {
    return hasFeature(typeid(FeatureT));
  }

  template<typename FeatureT>
  const FeatureT&
  getFeature() const {
    const FeatureSet& fset(featureSet());
    FeatureSet::const_iterator itr = fset.find(cast::typeName<FeatureT>());
    assert(itr != fset.end());
    assert(!itr->second.empty());
    return extractIDLFeature<FeatureT>(*(itr->second.begin()));
  }

  const OneTypeOfFeatures&
  getFeatures(const std::string& _type) const {
    const FeatureSet& fset(featureSet());
    FeatureSet::const_iterator itr = fset.find(_type);
    assert(itr != fset.end());
    assert(!itr->second.empty());
    return itr->second;
  }
  /// gets a feature set with only features comparable to \p _type
  const FeatureSet getFeatureSetComparableTo(const std::string& _type) const;

  template<typename FeatureT>
  const OneTypeOfFeatures&
  getFeatures() const {
    return getFeatures(cast::typeName<FeatureT>());
  }

//  const OptimizedFeatureSet& optimizedComparableFeatureSet() const;
//  const OptimizedFeatureSetWithRepetitions& optimizedComparableFeatureSetWithRepetitions() const;
  AbstractBindingWMRepresenter& bindingWMRepresenter() const {return bindingWMRepresenter;}
  cast::WorkingMemoryReaderProcess& component() const;
  //AbstractBinder& abstractBinder() const {return abstractBinder;}
  const PortMap& outPorts() const;
  const std::string& id() const {return id;}
  /// calls id()  (this is just for convenience elsewhere)
  const std::string& getID() const {return id();}
  const std::string& subarchitecture_id() const {return subarchitecture_id;}
  /// calls subarchitecture_id
  const std::string& subarchitectureID() const {return subarchitecture_id();}
  /// returns the version number (which the data had when it was loaded from WM)
  const unsigned int version() const {return version;}
  virtual BindingData::BindingProxyType type() const = 0;
  /// this only works if the proxy/union is a group (aborts if not)
  std::string getGroupDetailsID() const;
  const BindingFeatures::details::GroupDetails& getGroupDetails() const;
  const std::set<std::string>& getGroupMemberIDs() const;
  /// this only works if the proxy/union is a member of a group (aborts if not)
  const BindingFeatures::Singular& getSingularFeature() const;
  /// this only works if the proxy/union is a group (aborts if not)
  const BindingFeatures::Group& getGroupFeature() const;

private:
  mutable std::auto_ptr<std::set<std::string> > groupMemberIDs;
  mutable std::auto_ptr<cast::CachedCASTData<BindingFeatures::details::GroupDetails> > groupDetailsCache;
};

// forward devl
class LBindingUnion;

class LBindingProxy
  : public LocalBindingData
{
  boost::shared_ptr<const BindingData::BindingProxy> proxy;
  mutable cast::CachedCASTData<BindingData::BestUnionsForProxy> bestUnionsCache;
  mutable cast::CachedCASTData<BindingData::NonMatchingUnions> nonMatchingUnionsCache;
  mutable cast::CachedCASTData<BindingData::ProxyPorts> inPortsCache;
  mutable std::auto_ptr<std::set<std::string> > bestUnionsForProxySet;
  mutable int bestUnionsForProxySetLocalVersion;
  mutable std::auto_ptr<std::set<std::string> > nonMatchingUnionsSet;
  mutable int nonMatchingUnionsSetLocalVersion;
  mutable std::auto_ptr<BindingData::Ambiguity> ambiguity;

  //mutable std::set<std::string> boundProxyIDs;
public:

  LBindingProxy(const LBindingProxy& _lprox);
  LBindingProxy(AbstractBindingWMRepresenter& bindingWMRepresenter,
		const boost::shared_ptr<const BindingData::BindingProxy>& _proxy,
		const std::string& _id,
		const std::string& _subarchitecture_id,
		unsigned int _version);
  virtual ~LBindingProxy(){}
//  const boost::shared_ptr<const BindingData::BindingProxy>& proxyPtr() const {return proxy;}
  const BindingData::BindingProxy get() const {return *proxy;}
  const boost::shared_ptr<const BindingData::BindingProxy>& operator->() const {return proxy;}

public:

  const BindingData::BestUnionsForProxy& bestUnionsForProxy() const {return *bestUnionsCache;}
  const std::set<std::string>& bestUnionsForProxySet() const;
  const BindingData::NonMatchingUnions& nonMatchingUnions() const {return *nonMatchingUnionsCache;}
  const std::set<std::string>& nonMatchingUnionsSet() const;
  virtual const BindingData::ProxyPorts& inPorts() const {return *inPortsCache;}
  /// returns the outports just as they are on the WM version of the proxy
  virtual const BindingData::ProxyPorts& rawOutPorts() const {return proxy->outPorts;}


  /// returns true if bound
  bool bound() const {
#ifndef NDEBUG
    if(std::string(proxy->unionID).empty())
      assert(proxy->proxyState != BindingData::BOUND);
#endif // NDEBUG
    //return !(std::string(proxy->unionID).empty());
    return proxy->proxyState == BindingData::BOUND;
  }

  /// returns true if there is a best list
  bool scored() const {
    return (bestUnionsForProxy().unionIDs.length() != 0);
  }
  /// returns the state of the proxy.
  BindingData::BindingProxyState proxyState() const {return proxy->proxyState;}

  const LBindingUnion& bindingUnion() const throw(BindingException);
  const std::string& bindingUnionID() const;

  virtual BindingData::BindingProxyType type() const {return proxy->type;}

  /// returns a pointer an ambiguity, if there is one to be found
  BindingData::Ambiguity* ambiguity() const;
  /// returns true if the best list constains more than one proxy
  bool shouldHaveAmbiguity() const {
    ambiguityIDPostFix(); // dummy
    return bestUnionsForProxySet().size() > 1;
  }
private:
  mutable std::string bindingUnionID;
};

class LBindingUnion
  : public LocalBindingData
{
private:
  boost::shared_ptr<const BindingData::BindingUnion> union;
  mutable std::auto_ptr<std::set<std::string> > proxyIDs;
public:
  LBindingUnion(const LBindingUnion& _lunion);
  LBindingUnion(AbstractBindingWMRepresenter& _bindingWMRepresenter,
		const boost::shared_ptr<const BindingData::BindingUnion>& _union,
		const std::string& _id,
		const std::string& _subarchitecture_id,
		unsigned int _version);
  virtual ~LBindingUnion(){}
  //const boost::shared_ptr<const BindingData::BindingUnion>& unionPtr() const {return union;}
  const BindingData::BindingUnion& get() const {return *union;}
  const boost::shared_ptr<const BindingData::BindingUnion>& operator->() const {return union;}

  // later... const vector<const LBindingProxy* const> proxies() const;

  virtual const BindingData::ProxyPorts& inPorts() const {return union->inPorts;}
  virtual const BindingData::ProxyPorts& rawOutPorts() const {return union->outPorts;}

  const std::set<std::string>& proxyIDs() const;
  virtual BindingData::BindingProxyType type() const {return union->type;}
};

typedef boost::shared_ptr<const LBindingUnion> UnionPtr;
typedef boost::shared_ptr<const LBindingProxy> ProxyPtr;

} // namespace Binding

#endif
