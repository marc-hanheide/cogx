#ifndef BINDING_SPECIAL_EXTRACTORS_HPP_
#define BINDING_SPECIAL_EXTRACTORS_HPP_
#include <binding/utils/BasicExtractors.hpp>
#include <binding/utils/BindingUtils.hpp>
#include <binding/feature-utils/FeatureExtractor.hpp>

namespace Binding {

/// gets the union from a proxy
struct UnionFromProxyExtractor 
  : public AbstractExtractor<ProxyPtr> {
  virtual std::set<std::string> operator()(const ProxyPtr& _ptr) const {
    std::set<std::string> ret;
    ret.insert(_ptr->bindingUnionID());
    return ret;
  }
};

/// gets the bound proxies from a union
struct ProxyFromUnionExtractor 
  : public AbstractExtractor<UnionPtr> {
  virtual std::set<std::string> operator()(const UnionPtr& _ptr) const {
    std::set<std::string> ret(_ptr->proxyIDs());
    return ret;
  }
};

/// gets the bound proxies from a proxy directly
struct BoundProxyFromProxyExtractor 
  : public AbstractExtractor<ProxyPtr> {
  virtual std::set<std::string> operator()(const ProxyPtr& _ptr) const {
    std::set<std::string> ret;
    std::insert_iterator<std::set<std::string> > inserter = std::inserter(ret, ret.begin());
    for(unsigned int i = 0 ; i < (*_ptr)->proxyIDs.length() ; ++i)
      inserter = std::string((*_ptr)->proxyIDs[i]);
    return ret;
  }
};

/// matches ports
class AbstractPortMatcher {
  // The label of the port must match this regexp to expand ovr the
  // port (if not allocated, all labels match). the regexp syntax is
  // like in perl
  // (http://www.boost.org/doc/libs/1_35_0/libs/regex/doc/html/boost_regex/syntax/perl_syntax.html)
  std::auto_ptr<const boost::regex> regex;
public:
  AbstractPortMatcher(){}
  AbstractPortMatcher(const AbstractPortMatcher& _m) {
    if(_m.regex.get())
      regex = std::auto_ptr<boost::regex>(new boost::regex(*_m.regex));
  }
  AbstractPortMatcher(const boost::regex& _regex) : regex(new boost::regex(_regex)) {}
  /// creates a regex from the string
  AbstractPortMatcher(const std::string& _str) : regex(new boost::regex(_str)) {}
  //returns true if the port is a match
  bool match(const BindingData::ProxyPort& _port) const {
    if(!regex.get())
      return true;
    return boost::regex_match(std::string(_port.label),*regex);
  }
};

/// gets the proxies associated via outports
template<class LocalBindingDataPtrT>
struct OutPortsExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT>,
    public AbstractPortMatcher {
  OutPortsExtractor(){}
  OutPortsExtractor(const boost::regex& _regex) : AbstractPortMatcher(_regex) {}
  OutPortsExtractor(const std::string& _str) : AbstractPortMatcher(_str) {}
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    const BindingData::ProxyPorts& ports = _ptr->rawOutPorts();
    std::set<std::string> ret;
    for(unsigned int i = 0; i < ports.ports.length() ; ++i)
      if(match(ports.ports[i]))
	ret.insert(std::string(ports.ports[i].proxyID));
    return ret;
  }
private:
//  OutPortsExtractor(const OutPortsExtractor&){}
};

/// gets the proxies associated via inports
template<class LocalBindingDataPtrT>
struct InPortsExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT>,
    public AbstractPortMatcher {
  InPortsExtractor(){}
  InPortsExtractor(const boost::regex& _regex) : AbstractPortMatcher(_regex) {}
  InPortsExtractor(const std::string& _str) : AbstractPortMatcher(_str) {}
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    std::set<std::string> ret;
    const BindingData::ProxyPorts& ports = _ptr->inPorts();
    for(unsigned int i = 0; i < ports.ports.length() ; ++i)
      ret.insert(std::string(ports.ports[i].proxyID));
    return ret;
  }
};

/// extract the set of best union ids from a proxy
struct BestUnionsExtractor 
  : public AbstractExtractor<ProxyPtr> {
  virtual std::set<std::string> operator()(const ProxyPtr& _ptr) const {
    return _ptr->bestUnionsForProxySet();
  }
};

/// extract the set of nonmatching unions from a proxy
struct NonMatchingUnionsExtractor 
  : public AbstractExtractor<ProxyPtr> {
  virtual std::set<std::string> operator()(const ProxyPtr& _ptr) const {
    return _ptr->nonMatchingUnionsSet();
  }
};

/// gets the proxies associated via the Singular feature (which should
/// refer to the group proxy)
template<class LocalBindingDataPtrT>
struct GroupProxyFromSingularExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT> {
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    std::set<std::string> ret;
    const FeatureSetWithRepetitions& fset(_ptr->comparableFeatureSetWithRepetitions());
    FeatureSetWithRepetitions::const_iterator sing = fset.find(cast::typeName<BindingFeatures::Singular>());
    if(sing != fset.end()) {
      std::insert_iterator<std::set<std::string> > inserter = std::inserter(ret, ret.begin());
      foreach(OneTypeOfFeatures::value_type f, sing->second) { // f is an AbstractFeature
	inserter = std::string(extractIDLFeature<BindingFeatures::Singular>(f).groupID);
      }
    }
    return ret;
  }
};


/// gets the singulars belonging to a group
template<class LocalBindingDataPtrT>
struct SingularProxiesFromGroupExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT> {
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    return _ptr->getGroupMemberIDs();
  }
};


/// gets the proxies associated via the ThisFeature feature (which should
/// refer to the same proxy, or the bound unions). This is only really
/// useful for testingthe binder
template<class LocalBindingDataPtrT>
struct ProxyFromThisProxyIDExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT> {
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    std::set<std::string> ret;
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    FeatureSetWithRepetitions::const_iterator sing = fset.find(cast::typeName<BindingFeatures::ThisProxyID>());
    if(sing != fset.end()) {
      std::insert_iterator<std::set<std::string> > inserter = std::inserter(ret, ret.begin());
      foreach(OneTypeOfFeatures::value_type f, sing->second) { // f is an AbstractFeature
	inserter = std::string(extractIDLFeature<BindingFeatures::ThisProxyID>(f).thisProxyID);
      }
    }
    return ret;
  }
};


/// gets the proxies associated via the \p ExistingProxyID feature
template<class LocalBindingDataPtrT>
struct ProxyFromExistingProxyIDExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT> {
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    std::set<std::string> ret;
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    FeatureSetWithRepetitions::const_iterator sing = fset.find(cast::typeName<BindingFeatures::ExistingProxyID>());
    if(sing != fset.end()) {
      std::insert_iterator<std::set<std::string> > inserter = std::inserter(ret, ret.begin());
      foreach(OneTypeOfFeatures::value_type f, sing->second) { // f is an AbstractFeature
	inserter = std::string(extractIDLFeature<BindingFeatures::ExistingProxyID>(f).existingProxyID);
      }
    }
    return ret;
  }
};



} // namespace Binding

#endif //BINDING_SPECIAL_EXTRACTORS_HPP_

