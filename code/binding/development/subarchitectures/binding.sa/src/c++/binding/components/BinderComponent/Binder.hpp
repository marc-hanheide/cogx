#ifndef BINDING_BINDER_H_
#define BINDING_BINDER_H_

#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>
#include "binding/abstr/AbstractBinder.hpp"
//#include "dot/DotUtils.h"
#include "binding/utils/LocalClasses.hpp"


namespace Binding {

template<class T> const std::string& featureName();
template<class T> class Feature;

/// contains and keeps track of all unions
class Binder : public AbstractBinder {
public:
  Binder(const std::string &_id);
  virtual ~Binder();

  virtual void runComponent();  
  
  /// override start method to set change filters
  virtual void start();

  
  // moved to AbstractBinder:
  //  FeatureSet toFeatureSet(const BindingData::FeaturePointers& features);

public:
  
  /// triggered by a BestUnionsForProxy on WM
  void bindThisProxy(const cast::cdl::WorkingMemoryChange & _wmc);
  
  /// triggered by a BindingProxyDeletionTask
  void handleProxyDeletionTask(const cast::cdl::WorkingMemoryChange & _wmc);
  
  /// triggered by updates in a ProxyPorts object, correspondig to a
  /// inports being updated in a proxy. This updated must then be
  /// reflected in the union, which will be updated, and rescored
  /// accordingly.
  void updatePortsInUnion(const cast::cdl::WorkingMemoryChange & _wmc);

  /// updates the inports of the proxies referred to via the outports
  /// of the current proxy.
  void updateInports(const BindingData::ProxyPorts& _outPorts);
  
protected:
  virtual void taskAdopted(const std::string &_taskID);
  virtual void taskRejected(const std::string &_taskID);
  virtual void configure(std::map<std::string,std::string>& _config);  
  
private:
  
  
//  void _createNewUnion(const std::string& _proxyID,
//		       const std::string& _bestUnionsID);
  void _createNewUnion(const LBindingProxy& _proxy);

  void _bindProxy(const LBindingProxy& _proxy, 
		  std::string _unionID);

  void _updateProxyInUnion(const LBindingProxy& _proxy, 
			   const std::string& _unionID);


  /// updates info in the proxy
  void _updateProxyPointers(const LBindingProxy& _proxy, 
			    const std::string& _unionID,
			    const BindingData::BindingUnion& _union);
  
  void _addProxyFeaturesToUnionFeatures(const BindingData::FeaturePointers& _proxyFeatures,
					      BindingData::FeaturePointers& _unionFeatures) const;

  void _addProxyPortsToUnionPorts(const LBindingProxy& _proxy, 
				  BindingData::BindingUnion& _new_union) const;

  void _removeProxyFromUnion(const LBindingProxy& _proxy, 
			     const std::string& _unionID);
  
  // ...
  void _checkRelationsAndTriggerRescoring(const BindingData::ProxyPorts& _inports, const BindingData::ProxyPorts& _outports);

  void removeInports(const BindingData::ProxyPorts& _ports);
};

} // namespace Binding

#endif // BINDING_BINDER_H_
