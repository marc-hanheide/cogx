#ifndef COMA_TESTER_HPP_
#define COMA_TESTER_HPP_

#include <map>
#include <set>
#include <binding/utils/GraphLoader.hpp>
#include <binding/utils/Predicates.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/idl/BindingFeatures.hh>
#include <binding/BindingException.hpp>
#include <binding/feature-utils/AbstractFeature.hpp>
#include <binding/components/TesterMonitor/TesterMonitor.hpp>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <cast/core/CASTCore.hpp>


namespace Binding {

/// a monitor which creates proxies and then does some simple basic
/// sanity testing.
class ComaTester : 
    public TesterMonitor
{
  
public:
  ComaTester(const std::string &_id);

  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string> & _config);
  
protected:
  void bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingUnionUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc);


  void taskAdopted(const std::string& _taskID){}
  void taskRejected(const std::string& _taskID){}
  void runComponent();
    
  /// tests if the binding is now complete and exits with a success
  /// signal if so.
  void testCompleteness();

private:
  void addTwoProxiesAndOneRelation();

}; // ComaTester
} // namespace Binding

#endif // COMA_TESTER_HPP_

