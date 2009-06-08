#ifndef BINDING_GRAPH_LOADER_HPP_
#define BINDING_GRAPH_LOADER_HPP_
#include <binding/utils/Extractors.hpp>
#include <binding/utils/Misc.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <functional>
#include <deque>
#include <boost/function.hpp>
#include <boost/assign.hpp>
#include <boost/regex.hpp>
#include <boost/ref.hpp>
#include <map>
#include <iterator>

namespace Binding {
  
/// calls and stores the results of extractors
template <typename ExtractorT> 
class extractor_result_collector
{
  const ExtractorT& m_extractor;
  std::set<std::string> m_ids;
public:
  extractor_result_collector(const ExtractorT& _extractor) : m_extractor(_extractor) {}
  const std::set<std::string>& ids() {return m_ids;}
  void operator()(const std::pair<const std::string, typename ExtractorT::local_ptr_type>& _arg) {
    std::set<std::string> new_ids(m_extractor(_arg.second));
    m_ids.insert(new_ids.begin(), new_ids.end());
  }
};

// concenience factory function
template<typename ExtractorT>
extractor_result_collector<ExtractorT> 
result_collector(const ExtractorT& _extractor) {
  return extractor_result_collector<ExtractorT>(_extractor);
}


/// The handler loads proxies and unions according to extractors that
/// follow different links in the graph of unions and proxies on the
/// binding WM. The reslts are returned as sets. In order to avoid
/// weird situations when WM is updated while loading is partially
/// finished, the handler keeps a local copy of all proxies and
/// unions. The idea is that all proxies and unions can later be
/// checked for consistency w.r.t. to the WM. If inconsistent, it
/// wouldn't be enough to refresh all proxies and unions since they
/// may actually have new links that should have been followed.
struct 
BindingGraphHandler {
  BindingGraphHandler(AbstractBindingWMRepresenter& _component) : m_component(_component){}
  ProxySet allProxiesFromWM() {
    std::vector<boost::shared_ptr<const cast::CASTData<BindingData::BindingProxy> > > ptrs;
#warning very inefficient way of loading data
//    assert(!m_component.bindingSubarchID().empty());
    if(m_component.bindingSubarchID().empty())
      m_component.setBindingSubarchID(m_component.component().getSubarchitectureID());
    m_component.component().getWorkingMemoryEntries<BindingData::BindingProxy>(m_component.bindingSubarchID(),0,ptrs); // sets ptrs
    std::set<std::string> ids = 
      for_all(ptrs,insert_getID<boost::shared_ptr<const cast::CASTData<BindingData::BindingProxy> > >()).m_set;
    assert(ids.size() == ptrs.size());
    ProxySet ret = loadProxies(ids);
    return ret;
  }

  UnionSet allUnionssFromWM() {
    std::vector<boost::shared_ptr<const cast::CASTData<BindingData::BindingUnion> > > ptrs;
#warning very inefficient way of loading data
    m_component.component().getWorkingMemoryEntries<BindingData::BindingUnion>(m_component.bindingSubarchID(),0,ptrs); // sets ptrs
    std::set<std::string> ids = 
      for_all(ptrs,insert_getID<boost::shared_ptr<const cast::CASTData<BindingData::BindingUnion> > >()).m_set;
    UnionSet ret = loadUnions(ids);
    return ret;
  }

  /// returns all proxies locally stored in the GraphLoader
  const ProxySet& allLoadedProxies() {
    return m_allLoadedProxies;
  }

  /// returns all unions locally stored in the GraphLoader
  const UnionSet& allLoadedUnions() {
    return m_allLoadedUnions;
  }
  
  ProxySet extractProxiesFromProxies(const ProxySet& _proxies, 
				     const AbstractExtractor<ProxyPtr>& _fromProxyExtractor ) {
    std::set<std::string> ids = for_all(_proxies,result_collector(_fromProxyExtractor)).ids();
    ProxySet ret = loadProxies(ids);
    return ret;
  }
  
  UnionSet extractUnionsFromProxies(const ProxySet& _proxies, 
				    const AbstractExtractor<ProxyPtr>& _fromProxyExtractor = UnionFromProxyExtractor()) {
    std::set<std::string> ids = for_all(_proxies,result_collector(_fromProxyExtractor)).ids();
    UnionSet ret = loadUnions(ids);
    return ret;
  }
  
  UnionSet extractUnionsFromProxies(const ProxyPtr& _proxy, 
				    const AbstractExtractor<ProxyPtr>& _fromProxyExtractor = UnionFromProxyExtractor()) {
    //std::set<std::string> ids = for_all(_proxies,result_collector(_fromProxyExtractor)).ids();
    ProxySet pset; 
    pset.insert(std::make_pair(_proxy->id(),_proxy));
    UnionSet ret = extractUnionsFromProxies(pset, _fromProxyExtractor);
    //UnionSet ret = loadUnions(boost::assign::map_list_of(_proxy->id(),_proxy));
    return ret;
  }
  
  UnionSet extractUnionsFromUnions(const UnionSet& _unions, 
				   const AbstractExtractor<UnionPtr>& _fromUnionExtractor) {
    std::set<std::string> ids = for_all(_unions,result_collector(_fromUnionExtractor)).ids();
    UnionSet ret = loadUnions(ids);
    return ret;
  }

  ProxySet extractProxiesFromUnions(const UnionSet& _unions, 
				    const AbstractExtractor<UnionPtr>& _fromUnionExtractor = ProxyFromUnionExtractor()) {
    std::set<std::string> ids = for_all(_unions,result_collector(_fromUnionExtractor)).ids();
    ProxySet ret = loadProxies(ids);
    return ret;
  }
      
public:

  ProxyPtr& loadProxy(const std::string& _id) {
    ProxySet::iterator itr = m_allLoadedProxies.find(_id);
    if(itr == m_allLoadedProxies.end()) {
      try {
	itr = m_allLoadedProxies.insert(make_pair(_id,m_component.m_proxyLocalCache.get(_id).getLocalPtr())).first;
      } 
      catch (const cast::DoesNotExistOnWMException& _e ) {
	m_component.component().log(std::string("while loading a proxy in BindingGraphHandler: (") + _e.what() +")");
	throw _e;
      }
    }
    return itr->second;
  }
  
private:
  /// a helper function (helping the compiler that is...)
  ProxyPtr& _loadProxy(boost::reference_wrapper<const std::string> _id) {
    return loadProxy(_id);
  }

public:
  /// returns a function pointer to \p loadProxy of \p this object
  boost::function<ProxyPtr& (boost::reference_wrapper<const std::string>)> loadProxyFctPtr() const {
    return std::bind1st(mem_fun(&BindingGraphHandler::_loadProxy), this);
  }
  
  UnionPtr& loadUnion(const std::string _id) {
    UnionSet::iterator itr = m_allLoadedUnions.find(_id);
    if(itr == m_allLoadedUnions.end()) {
      try {
	itr = m_allLoadedUnions.insert(make_pair(_id,m_component.m_unionLocalCache.get(_id).getLocalPtr())).first;
      } 
      catch (const cast::DoesNotExistOnWMException& _e ) {
	m_component.component().log(std::string("while loading a union in BindingGraphHandler: (") + _e.what() +")");
	throw _e;
      }
    }
    return itr->second;
  }

private:
  /// a helper function (helping the compiler that is...)
  UnionPtr& _loadUnion(boost::reference_wrapper<const std::string> _id) {
    return loadUnion(_id);
  }

public:
  /// returns a function pointer to \p loadProxy of \p this object
  boost::function<UnionPtr& (boost::reference_wrapper<const std::string>)> loadUnionFctPtr() const {
    return std::bind1st(mem_fun(&BindingGraphHandler::_loadUnion), this);
  }

private:  
  struct loadProxiesFct {
    ProxySet m_proxies;
    BindingGraphHandler& m_handler;
    loadProxiesFct(BindingGraphHandler& _handler) : m_handler(_handler) {}
    ProxyPtr& operator()(const std::string& _id) {
      ProxyPtr& ptr(m_handler.loadProxy(_id));
      m_proxies.insert(make_pair(_id,ptr));
      return ptr;
    }
  };
  struct loadUnionsFct {
    UnionSet m_unions;
    BindingGraphHandler& m_handler;
    loadUnionsFct(BindingGraphHandler& _handler) : m_handler(_handler) {}
    UnionPtr& operator()(const std::string& _id) {
      UnionPtr& ptr(m_handler.loadUnion(_id));
      m_unions.insert(make_pair(_id,ptr));
      return ptr;
    }
  };
public:
  
  /// loads a number of proxies from a set of ids (calls \p loadProxy)
  //
  // nah: made templated for any iterable containing strings... hope it makes sense
  template <class StringContainer>
  ProxySet loadProxies(const StringContainer & _ids) {
    return for_each(_ids.begin(),_ids.end(),loadProxiesFct(*this)).m_proxies;
  }


//   /// loads a number of proxies from a set of ids (calls \p loadProxy)
//   ProxySet loadProxies(const std::set<std::string>& _ids) {
//     return for_each(_ids.begin(),_ids.end(),loadProxiesFct(*this)).m_proxies;
//   }

  /// loads a number of proxies from a set of ids (calls \p loadProxy)
  ProxySet loadProxies(const BindingData::WorkingMemoryIDList& _ids) {
    std::set<std::string> ids;
    std::insert_iterator<std::set<std::string> > 
      inserter = std::inserter(ids,ids.begin());
    for(unsigned int i = 0 ; i < _ids.length() ; ++i) 
      inserter = std::string(_ids[i]);
    return loadProxies(ids);
  }
  
  /// loads one proxy from an id (calls \p loadProxies)
  ProxySet loadProxies(const std::string& _id) {
    loadProxiesFct loader(*this);
    loader(_id);
    return loader.m_proxies;
  }
  
  /// refreshes all loaded proxies from WM
  void reloadAllLoadedProxies() {
    std::set<std::string> ids = for_all(m_allLoadedProxies,
					insert_first<std::pair<const std::string,ProxyPtr> >()).m_set;
    m_allLoadedProxies.clear();
    loadProxies(ids);
  }

  /// loads a number of proxies from a set of ids (calls \p loadProxy)
  UnionSet loadUnions(const std::set<std::string>& _ids) {
    return for_each(_ids.begin(),_ids.end(),loadUnionsFct(*this)).m_unions;
  }

  /// refreshes all loaded unions from WM
  void reloadAllLoadedUnions() {
    std::set<std::string> ids = for_all(m_allLoadedUnions,
					insert_first<std::pair<const std::string,UnionPtr> >()).m_set;
    m_allLoadedUnions.clear();
    loadUnions(ids);
  }
  
  /// calls \p reloadAllLoadedUnions and \p reloadAllLoadedProxies
  void reloadAllLoadedData() {
    reloadAllLoadedProxies();
    reloadAllLoadedUnions();
  }
  
  /// clears the loaded data completely
  void purgeAllLoadedData() {
    m_allLoadedUnions.clear();
    m_allLoadedProxies.clear();
  }


  
private:
  /// contains already expanded proxies (used to avoid reloading, so that all parts of the graphs are frozen in time)
  UnionSet m_allLoadedUnions;
  /// contains already expanded unions (used to avoid reloading, so that all parts of the graphs are frozen in time)
  ProxySet m_allLoadedProxies; 
  /// contains the resulting proxies
  AbstractBindingWMRepresenter& m_component;
  /// not CopyConstructible
  BindingGraphHandler(const BindingGraphHandler&);
  /// not assignable
  BindingGraphHandler& operator=(const BindingGraphHandler&);
};


} // namespace Binding
#endif //BINDING_GRAPH_LOADER_HPP_

