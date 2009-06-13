#ifndef BINDING_ABSTRACT_BINDER_H_
#define BINDING_ABSTRACT_BINDER_H_

//#include "binding/utils/BindingScoreUtils.hpp"
//#include "binding/feature-utils/Features.hpp"
#include "AbstractBindingWMRepresenter.hpp"
//#include "binding/utils/LocalClasses.hpp"
//#include "BindingData.hpp"
//#include "cast/core/CASTDataCache.hpp"
//#include <cast/core/CASTDataLocalCache.hpp>
#include "cast/architecture/ChangeFilterFactory.hpp"

#include <cast/architecture/PrivilegedManagedProcess.hpp>
//#include <boost/logic/tribool.hpp>
//#include <boost/shared_ptr.hpp>
//#include <map>
//#include <set>


namespace Binding {

class AbstractBinder: 
    public cast::PrivilegedManagedProcess,    
    public AbstractBindingWMRepresenter {

public:
  /// override start method to set change filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string>& _config);

  /// locks the entry so that only the calling component can overwrite (i.e. \p cast::cdl::WorkingMemoryPermissions = LOCKED_O)
  template<typename T>
  void acquireEntry(const cast::cdl::WorkingMemoryChange & _wmc) {
    loadBindingDataFromWM<T>(_wmc); // for consistency
    lockEntry(std::string(_wmc.address.id),std::string(_wmc.address.subarchitecture), cast::cdl::LOCKED_O);
  }

  /// returns the address of the singular binder token
  virtual cast::cdl::WorkingMemoryAddress _binderTokenAddress() const {
    cast::cdl::WorkingMemoryAddress a;
    a.subarchitecture = CORBA::string_dup(subarchitectureID().c_str());
    a.id = CORBA::string_dup(BindingData::binderTokenID);
    return a;
  }

  /// acquires a token that will be owned by either the binding SA or a monitor
  virtual void acquireBinderToken();
  /// releases the token that will be owned by either the binding SA or a monitor
  virtual void releaseBinderToken();

  /// returns true iff the component owns the bindr token
  bool hasBinderToken() const
  {
    return tokens().find(_binderTokenAddress()) != tokens().end();
  }

  /// returns the address of the singular binder token
  cast::cdl::WorkingMemoryAddress _internalBindingTokenAddress() const {
    cast::cdl::WorkingMemoryAddress a;
    a.subarchitecture = CORBA::string_dup(subarchitectureID().c_str());
    a.id = CORBA::string_dup(BindingData::internalBindingTokenID);
    return a;
  }
  
  /// acquires a token that will be owned by either the binding SA or a monitor
  virtual void acquireInternalBindingToken();
  /// releases the token that will be owned by either the binding SA or a monitor
  virtual void releaseInternalBindingToken();

  /// returns true iff the component owns the bindr token
  bool hasInternalBindingToken() const
  {
    return tokens().find(_internalBindingTokenAddress()) != tokens().end();
  }

  /// returns the address of the singular binder token
  virtual cast::cdl::WorkingMemoryAddress _binderLockTokenAddress() const {
    cast::cdl::WorkingMemoryAddress a;
    a.subarchitecture = CORBA::string_dup(subarchitectureID().c_str());
    a.id = CORBA::string_dup(BindingData::binderLockTokenID);
    return a;
  }


  /// acquires a token that will be owned by either the binding SA or a monitor
  virtual void acquireBinderLockToken();
  /// releases the token that will be owned by either the binding SA or a monitor
  virtual void releaseBinderLockToken();

  /// returns true iff the component owns the bindr token
  bool hasBinderLockToken() const
  {
    return tokens().find(_binderLockTokenAddress()) != tokens().end();
  }


private:
  friend class BindingScorer;
  friend class AbstractMonitor;
  virtual cast::cdl::WorkingMemoryAddress _binderTokenTokenAddress() const {
    cast::cdl::WorkingMemoryAddress a;
    a.subarchitecture = CORBA::string_dup(subarchitectureID().c_str());
    a.id = CORBA::string_dup(BindingData::binderTokenTokenID);
    return a;
  }
  /// acquires a token that will be owned by either the binding SA or a monitor
  void acquireBinderTokenToken() 
  {
    acquireToken(_binderTokenTokenAddress());
  }
  /// releases the token that will be owned by either the binding SA or a monitor
  void releaseBinderTokenToken() 
  {
    releaseToken(_binderTokenTokenAddress());
  }

  /// returns true iff the component owns the bindr token
  bool hasBinderTokenToken() const
  {
    return tokens().find(_binderTokenTokenAddress()) != tokens().end();
  }

  
protected:
  AbstractBinder(const std::string &_id);
  virtual ~AbstractBinder();


  
protected:  
  
  void changeProxyState(const LBindingProxy& _proxy, 
			BindingData::BindingProxyState _newState);
  
public: 
  


};





} // namespace Binding

#endif // BINDING_ABSTRACT_BINDER_H_
