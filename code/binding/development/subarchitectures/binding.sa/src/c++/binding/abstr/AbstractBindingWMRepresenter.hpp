#ifndef BINDING_ABSTRACT_BINDING_WM_REPRESENTER_H_ 
#define BINDING_ABSTRACT_BINDING_WM_REPRESENTER_H_ 

#include <cast/core/CASTDataCache.hpp>
#include <cast/core/CASTDataLocalCache.hpp>
#include "binding/utils/LocalClasses.hpp"
//#include "binding/feature-utils/Features.hpp"
#include "binding/feature-utils/FeatureLoader.hpp"

#include <boost/shared_ptr.hpp>
#include <map>
#include <set>


namespace Binding {

/// compares and sorts WM adresses
struct WMALess {
  bool operator()(const cast::cdl::WorkingMemoryAddress& _a1,
		  const cast::cdl::WorkingMemoryAddress& _a2) const{
    std::string id1(_a1.m_id);
    std::string id2(_a2.m_id);
    if(id1 != id2) return id1 < id2;
    std::string sa1(_a1.m_subarchitecture);
    std::string sa2(_a2.m_subarchitecture);
    return sa1 < sa2;
  }
};

typedef std::set<cast::cdl::WorkingMemoryAddress, WMALess> WMASet;


/// Contains the datatypes needed to store binding WM locally
class AbstractBindingWMRepresenter {
  mutable cast::WorkingMemoryReaderProcess& m_component;
protected:
  AbstractBindingWMRepresenter(cast::WorkingMemoryReaderProcess& m_component);
  virtual ~AbstractBindingWMRepresenter() { };
  
public:
  /// returns m_component
  cast::WorkingMemoryReaderProcess& component() const {return m_component;}
protected:

  /// stores local copy of proxies (maps from CAST ID)
//  std::map<std::string,LBindingProxy> m_proxies;
  /// stores local copy of bindings (maps from CAST ID)
  //std::map<std::string,LBindingUnion> m_unions;
  /// Maps from a binding to the bound proxies (CAST IDs)
  std::map<std::string,std::set<std::string> > m_uni2prox;
  /// Maps from proxy to the binding of that proxy (CAST IDs)
  std::map<std::string,std::string> m_prox2uni;
  
  /// extracts local proxy from CAST ID
//  const LBindingProxy& _localProxy(const std::string&) const;
  /// extracts local binding from CAST ID
//  const LBindingUnion& _localUnion(const std::string&) const;
  friend class LocalBindingData;
  /// Implemented in \p FeatureProperties.cpp just to keep things that
  /// need to be changed by anyone adding features in one place. It
  /// returns a shared pointer to a AbstractFeature if \p _feat can be
  /// translated into such. Otherwise it throws a \p runtime_error.
  boost::shared_ptr<AbstractFeature>
  toFeature(const BindingData::FeaturePointer& _feat);

public:  
  template<class T>
  const boost::shared_ptr<const T> loadBindingDataFromWM(const cast::cdl::WorkingMemoryChange & _wmc) {
    return loadBindingDataFromWM<T>(_wmc.m_address);
  }
  template<class T>
  const boost::shared_ptr<const T> loadBindingDataFromWM(const cast::cdl::WorkingMemoryAddress & _address) {
    return loadBindingDataFromWM<T>(std::string(_address.m_subarchitecture), std::string(_address.m_id));
  }
  template<class T>
  const boost::shared_ptr<const T> loadBindingDataFromWM(const CORBA::String_member& _str) {
    return loadBindingDataFromWM<T>(std::string(_str));
  }
  template<class T>
  const boost::shared_ptr<const T> loadBindingDataFromWM(const _CORBA_String_element& _str) {
    return loadBindingDataFromWM<T>(std::string(_str));
  }
  template<class T>
  const boost::shared_ptr<const T> loadBindingDataFromWM(const std::string& _id) {
    return loadBindingDataFromWM<T>(bindingSubarchID(), _id);
  }
  template<class T>
  const boost::shared_ptr<const T> loadBindingDataFromWM(const std::string& _subarchitecture, const std::string& _id) {
    //log("loadBindingDataFromWM: _subarchitecture: " + _subarchitecture + " _id: " + _id);
    boost::shared_ptr<const cast::CASTData<T> > data_ptr(m_component.getWorkingMemoryEntry<T>(_id,_subarchitecture));
    //log("==================== UGLY TEST ++++++++++++++++++++");
    //boost::shared_ptr<const CASTData<T> >* data_ptr = new boost::shared_ptr<const CASTData<T> >(getWorkingMemoryEntry<T>(_id));
    //boost::shared_ptr<const CASTData<T> > data_ptr = getWorkingMemoryEntry<T>(_id);
    return data_ptr->getData();  
  }
protected:
  /// supertype for the local cache translator functor classes
  struct local_converter {
    AbstractBindingWMRepresenter& m_bindingWMRepresenter;
    local_converter(AbstractBindingWMRepresenter& _bindingWMRepresenter) 
      : m_bindingWMRepresenter(_bindingWMRepresenter) {}
  };
  
  /// translator functor for proxies into local representation
  class local_proxy_converter : local_converter {
  public:
    local_proxy_converter(AbstractBindingWMRepresenter& _bindingWMRepresenter) 
      : local_converter(_bindingWMRepresenter) {}
    LBindingProxy operator()(const boost::shared_ptr<const BindingData::BindingProxy>& _proxy, 
			     const std::string& _id, 
			     const std::string& _subarchID,
			     unsigned int _version) const 
    {
      return LBindingProxy(m_bindingWMRepresenter, _proxy, _id, _subarchID, _version);
    }
  };
  
  /// translator functor for unions into local representation
  class local_union_converter : local_converter {
  public:
    local_union_converter(AbstractBindingWMRepresenter& _bindingWMRepresenter) 
      : local_converter(_bindingWMRepresenter) {}
    LBindingUnion operator()(const boost::shared_ptr<const BindingData::BindingUnion>& _union, 
			     const std::string& _id, 
			     const std::string& _subarchID,
			     const unsigned int _version) const 
    {
      return LBindingUnion(m_bindingWMRepresenter, _union, _id, _subarchID, _version); 
    }
  };
  
  /// will lock the item (which is here interpreted as a token) at the
  /// address (and wait until it is indeed locked)
  void acquireToken(const cast::cdl::WorkingMemoryAddress&);
  /// will unlock the item at the address (and wait until it is indeed
  /// locked)
  void releaseToken(const cast::cdl::WorkingMemoryAddress&);
  /// releases all tokens
  void releaseAllTokens();
  
private:
protected:

  
public:    
  cast::CASTDataLocalCache<BindingData::BindingProxy,LBindingProxy,local_proxy_converter> m_proxyLocalCache;
  cast::CASTDataLocalCache<BindingData::BindingUnion,LBindingUnion,local_union_converter> m_unionLocalCache;

  /// loads using the cache. Returns null if the loading was not
  /// successful
  const LBindingProxy* maybeLoadProxy(const std::string& _proxyID);
  /// loads using the cache. Returns null if the loading was not
  /// successful
  const LBindingUnion* maybeLoadUnion(const std::string& _proxyID);

  FeatureLoader m_featureLoader;

  std::string m_bindingSubarchID;
  const std::string& bindingSubarchID() const {return m_bindingSubarchID;}
public:
  void setBindingSubarchID(const std::string& _str) {
    m_bindingSubarchID = _str;
    m_proxyLocalCache.setSubarchitectureID(_str);
    m_unionLocalCache.setSubarchitectureID(_str);
  }

private:
  /// a list of the acquired tokens
  WMASet m_tokens;

public:
  const WMASet& tokens() const {return m_tokens;}
};


} // namespace Binding

namespace cast {

template<>
const Binding::LBindingProxy&
cast::CASTDataLocalCache<BindingData::BindingProxy,Binding::LBindingProxy,Binding::AbstractBindingWMRepresenter::local_proxy_converter>::operator[](const std::string& _id);

template<>
const Binding::LBindingUnion&
cast::CASTDataLocalCache<BindingData::BindingUnion,Binding::LBindingUnion,Binding::AbstractBindingWMRepresenter::local_union_converter>::operator[](const std::string& _id);


}

#endif //  BINDING_ABSTRACT_BINDING_WM_REPRESENTER_H_ 
