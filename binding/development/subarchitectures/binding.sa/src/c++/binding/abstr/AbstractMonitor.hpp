#ifndef BINDER_ABSTRACT_MONITOR_H_
#define BINDER_ABSTRACT_MONITOR_H_
#include <set>
//#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include "cast/cdl/CAST.hh"
#include "cast/architecture/WorkingMemoryChangeReceiver.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"
//#include "binding/idl/BindingData.hh"
//#include "binding/idl/BindingFeaturesCommon.hh"
//#include "binding/ontology/BindingFeatureOntology.hpp"
//#include "feature-specialization/FeatureProperties.hpp"
#include "binding/BindingException.hpp"
#include "binding/abstr/AbstractBinder.hpp"
#include <binding/idl/BindingQueries.hh>

//#include "binding/abstr/AbstractBindingWMRepresenter.hpp"
//#include "feature-utils/AbstractFeature.hpp"
#include <memory>


namespace Binding {
  
//forward decl
class CopyCatBindingMonitor;

class AbstractMonitor : public AbstractBinder  {
  friend class BindingGroupManager;
  friend class CopyCatBindingMonitor;
protected:
  /// The constructor...
  AbstractMonitor(const std::string& _id);

public:
  /// starts a new proxy (as a group, automatically stores the group feature)
  /// \p _groupSize is the maximum size of the group, if 0, then it
  /// means the group is unbounded
  void startNewGroupProxy(unsigned int _groupSize);
  
  /// starts a new proxy (as a basic type, i.e. not a group)
  void startNewBasicProxy() {
    _startNewProxy(BindingData::BASIC);
  }
  
  
  /// starts a new proxy of relation type
  void startNewRelationProxy() {
    _startNewProxy(BindingData::RELATION);
  }
  
  /// Adds an outport to the current proxy. The outport will be used
  /// for scoring. This should only be done for relation
  /// proxies. Calling \p addOutPortToCurrentProxy N times means an
  /// N-ary relation. The label is used to specify the role of the
  /// port in the relation, e.g. "to" or "from". If it's a
  /// bidirectional relation, then let both ports have the same label.
  void addOutPortToCurrentProxy(const std::string _proxyID, 
				const std::string _portLabel);
  
  /// loads an existing proxy and marks all its features for
  /// deletion.
  /// \param _proxyAddr 
  /// \param _deleteTheseTypes
  /// features of these types will be
  /// deleted from the existing binding proxy, for example, if you
  /// want to change the colour of an object, do not keep it, and then
  /// add new colour. Possibly we need something more sophisticated
  /// here when updating features with more than one value.
  void changeExistingProxy(const std::string& _proxyAddr, const std::set<std::string>& _deleteTheseFeatureTypes = std::set<std::string>());
  
  /// Simply deletes the proxy and all its features
  void deleteExistingProxy(const std::string& _proxyAddr);

  /// cancels the creation of a proxy, deletes all created
  /// features up to this point and makes it possible to start a new
  /// proxy.
  void cancelCurrentProxy();

  /// Stores a feature to WM and adds the adress to the set of features pointers of the proxy
  template<class FeatureType>
  BindingData::FeaturePointer addFeatureToCurrentProxy(const FeatureType& _feature, BindingFeaturesCommon::TruthValue _truthValue = BindingFeaturesCommon::POSITIVE) {
    if(m_currentlyBuiltProxy.get() == NULL) {
      throw(BindingException("Attempt to add feature to m_proxyFeatures before initiating it with startNewProxy"));
    }
    BindingData::FeaturePointer ptr = _storeFeature(_feature,_truthValue);
    m_currentlyBuiltProxy->m_proxyFeatures.length(m_currentlyBuiltProxy->m_proxyFeatures.length() + 1); 
    m_currentlyBuiltProxy->m_proxyFeatures[m_currentlyBuiltProxy->m_proxyFeatures.length() - 1] = ptr;
    return ptr;
  }
  
  /// deletes one feature from the current proxy (suitable for when
  /// updating proxies and only one feature should be updated)
  void deleteFeatureFromCurrentProxy(const BindingData::FeaturePointer&);

  /// adds the source ID to the proxy. Make sure your instantiated monitor sets m_sourceID first.
  BindingData::FeaturePointer addSourceIDToCurrentProxy();
  
  /// Adds the creation time to the proxy
  BindingData::FeaturePointer addCreationTimeToCurrentProxy();

  /// specifies that the proy should be scored base on unions stemming
  /// from a particular subarch (or vice versa, if negated)
  BindingData::FeaturePointer addOtherSourceIDToCurrentProxy(std::string _id, BindingFeaturesCommon::TruthValue _truthValue);
  
  BindingData::FeaturePointer addSalienceToCurrentProxy(const BindingFeaturesCommon::StartTime& _start,
							const BindingFeaturesCommon::EndTime& _end);
  
  /// Adds a single time instant as salience time (for utterances,
  /// when words "this is" are uttered, for example)
  BindingData::FeaturePointer addSalienceToCurrentProxy(const FrameworkBasics::BALTTime& _time);
  /// Adds the current time as a salience time instant
  BindingData::FeaturePointer addSalienceToCurrentProxy();
  
  
  /// stores the current proxy onto WM and returns the address. It
  /// also resets m_currentlyBuiltProxy to null.
  std::string storeCurrentProxy(bool _storeSystemFeatures = true);
  
   /// Adds a relation from proxy on WM address ID _from to
   /// proxy on WM address ID \p _to with \p _label to denote the type
   /// of relation.
  std::string addSimpleRelation(const std::string& _from, const std::string& _to, const std::string& _label);
  

  
  ///causes the proxy to be hypothetical, and will not be bound to anything.  
  void makeCurrentProxyHypothetical();
  
  /// After all new proxies are added and relations between them
  /// defined it is time to bind them. This stores a
  /// BindTheseProxies object onto the WM and resets \p
  /// m_unboundProxyAdresses to an empty list 
  void bindNewProxies();
  
  /// defines the ID of the monitored SA, must be updated by derived class!!!
  std::string m_sourceID; 


  /// The ID of the latest added sourceID, must be added to
  /// OtherSorceID so that the scorer can avoid scoring it with a
  /// SourceID of the same proxy
  std::string m_sourceIDAddress;

  /// the ID of the build proxy.
  const std::string& currentProxyID() const {return m_currentProxyID;}

 
private:
  
  /// defines the ID of binding SA, must be set via configure
  std::string m_bindingSA;

  /// The proxy which is currently being built up by features being added to it (and WM)  
  std::auto_ptr<BindingData::BindingProxy> m_currentlyBuiltProxy;  

  /// if currently built proxy is a new one, this is false
  bool m_updatingProxy;  
  
  /// the ID of the build proxy.
  std::string m_currentProxyID;  
  
  /// A list of proxies added to the WM but which have not yet been
  /// bound (bindNewProxies binds the proxies and empties the
  /// list)
  std::set<std::string> m_unboundProxyAddresses;

  BindingFeaturesCommon::TemporalFrameType m_temporalFrameOfCurrentProxy;
  
  /// stores the feature onto the WM and returns the CAST pointer
  template<class FeatureType>
  BindingData::FeaturePointer _storeFeature(const FeatureType& _feature, BindingFeaturesCommon::TruthValue _truthValue) {
    BindingData::FeaturePointer ret;
    FeatureType* feature_ptr = new FeatureType(_feature);
    feature_ptr->m_parent = defaultParentFeature();
    feature_ptr->m_parent.m_truthValue = _truthValue;
    ret.m_address = CORBA::string_dup(newDataID().c_str());
    static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());

    ret.m_type    = CORBA::string_dup(ontology.featureName(typeid(FeatureType)).c_str());
    ret.m_immediateProxyID = CORBA::string_dup(m_currentProxyID.c_str());
    addToWorkingMemory(std::string(ret.m_address), m_bindingSA, 
		       //std::string(ret.m_type), 
		       feature_ptr);
    
    return ret;
  }
  
  struct cmpFeaturePointer {
    bool operator()(const BindingData::FeaturePointer& _bfp1, const BindingData::FeaturePointer& _bfp2)
    {
      return(_bfp1.m_address < _bfp2.m_address);
    }
  };
  
  typedef std::set<BindingData::FeaturePointer,cmpFeaturePointer> FeaturePointerSet;
  
  /// features from an updated proxy must be deleted to avoid
  /// memory leaks
  FeaturePointerSet m_featuresToDelete;

private:
  /// deletes the features from WM. Only to be used when cancelling an
  /// unstored proxy, therefore private
  void _deleteFeatures(const FeaturePointerSet&);
  void _startNewProxy(const BindingData::BindingProxyType _proxyType, const std::string& _insistedID = "");


public:
  // calls _startNewProxy(const BindingData::BindingProxyType _proxyType);
  void startNewProxy(const BindingData::BindingProxyType _proxyType) {_startNewProxy(_proxyType);}

  // calls _startNewProxy(const BindingData::BindingProxyType _proxyType);
  void startNewProxy(const BindingData::BindingProxyType _proxyType, const std::string& _insistedID) {_startNewProxy(_proxyType, _insistedID);}

  
  virtual void configure(std::map<std::string,std::string> & _config);
  /// obs! must be called from instantiated monitors
  virtual void start();
  

private:
  /// updates the inports of the proxies referred to via the outports
  /// of the current proxy.
  /// \remark deprecated... now handled by the binder completely
  void updateInports(const BindingData::ProxyPorts& _outPorts);

private:
  /// returns the address of the token
  cast::cdl::WorkingMemoryAddress _bindTheseProxiesMonitorTokenAddress() const;
  /// waits for the token at \p BindingData::bindTheseProxiesMonitorTokenID
  void acquireBindTheseProxiesMonitorToken();
  /// releases for the token at \p BindingData::bindTheseProxiesMonitorTokenID
  void releaseBindTheseProxiesMonitorToken();
public:
  /// returns true if the token is in the possession of this component
  bool hasBindTheseProxiesMonitorToken() {return tokens().find(_bindTheseProxiesMonitorTokenAddress()) != tokens().end();}
private:  
  /// returns the address of the token
  cast::cdl::WorkingMemoryAddress _updateProxyMonitorTokenAddress() const;
  /// waits for the token at \p BindingData::bindTheseProxiesMonitorTokenID
  void acquireUpdateProxyMonitorToken();
  /// releases for the token at \p BindingData::bindTheseProxiesMonitorTokenID
  void releaseUpdateProxyMonitorToken();
  
  virtual cast::cdl::WorkingMemoryAddress _binderTokenAddress() const
  {
    cast::cdl::WorkingMemoryAddress a;
    a.m_subarchitecture = CORBA::string_dup(bindingSA().c_str());
    a.m_id = CORBA::string_dup(BindingData::binderTokenID);
    return a;
  }
  virtual cast::cdl::WorkingMemoryAddress _binderTokenTokenAddress() const
  {
    cast::cdl::WorkingMemoryAddress a;
    a.m_subarchitecture = CORBA::string_dup(bindingSA().c_str());
    a.m_id = CORBA::string_dup(BindingData::binderTokenTokenID);
    return a;
  }




public:
  /// returns true if the token is in the possession of this component
  bool hasUpdateProxyMonitorToken() {return tokens().find(_updateProxyMonitorTokenAddress()) != tokens().end();}



protected:
  /// Maps from proxy IDs to the address of the inportlist
  std::map<std::string,std::string> m_proxyID2inportsID;

  const std::string& getBindingSA() const {
    if(m_bindingSA == ""){ 
      throw(BindingException("Binding SA ID must be set."));
    }
    return m_bindingSA;
  }
  
  /// returns getBindingSA (just a less verbose interface)
  const std::string& bindingSA() const {
    return getBindingSA();
  }

  void setBindingSA(const std::string & _bindingSA) {
    m_bindingSA = _bindingSA;
  }
  
  BindingFeaturesCommon::ParentFeature defaultParentFeature() const; 
  
private:
  bool m_binderReady;
  /// sets m_binderReady to true
  void binderReadySignal(const cast::cdl::WorkingMemoryChange & _wmc);

  /// true if \p start() is called
  bool m_startCalled;
  friend class cast::MemberFunctionChangeReceiver<AbstractMonitor>;
  
public: 
  /// returns true if the binder has signalled it is ready for proxies
  /// etc. Don't attempt to do anything until it is ready.
  bool binderReady() const {return m_binderReady;}

private:
  /// for dealing with when \p BindingQueries::MakeProxyUnavailable
  /// are added to WM. Calls \pmakeProxyUnavailable() if the entry
  /// belongs to this monitor;
  void makeProxyUnavailableAdded(const cast::cdl::WorkingMemoryChange & _wmc);
protected:
  /// called by \p makeProxyUnavailableAdded if the \p
  /// \p BindingQueries::MakeProxyUnavailable belongs to this monitor. It will simply
  /// call \p deleteExistingProxy() right away.
  /// \remark override this function if immediate deletion is not appropriate
  virtual void makeProxyUnavailable(const BindingQueries::MakeProxyUnavailable&);

private:
  /// a set of all proxies that have been created (and not yet
  /// deleted) by this monitor.
  std::set<std::string> m_ownedProxyIDs;
public:
  /// returns ref to \p m_ownedProxyIDs
  const std::set<std::string>& ownedProxyIDs() {return m_ownedProxyIDs;}
  
}; // Abstract monitor
} // namespace Binding

#endif // BINDER_ABSTRACT_MONITOR_H_
