#ifndef BINDING_ABSTRACT_ACTIVE_BINDING_WM_READER_H_ 
#define BINDING_ABSTRACT_ACTIVE_BINDING_WM_READER_H_

#include <cast/architecture/ManagedProcess.hpp>

#include <map>
#include <set>

namespace Binding {

/// Abstract class containing methods that will be called when binding
/// WM updates. 
class AbstractActiveBindingWMReader :
    public cast::ManagedProcess
{
public:

  /// override start method to set change filters
  virtual void start();
    
  /// Called when a proxy is added to the wm
  virtual void proxyAdded(const cast::cdl::WorkingMemoryChange & _wmc) = 0;
  /// Called when a proxy is overwritten in the wm
  virtual void proxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc) = 0;
  /// Called when a proxy is deleted from the wm
  virtual void proxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc) = 0;

  /// Called when a proxy is added to the wm
  virtual void relationAdded(const cast::cdl::WorkingMemoryChange & _wmc) = 0;
  /// Called when a proxy is overwritten in the wm
  virtual void relationUpdated(const cast::cdl::WorkingMemoryChange & _wmc) = 0;
  /// Called when a proxy is deleted from the wm
  virtual void relationDeleted(const cast::cdl::WorkingMemoryChange & _wmc) = 0;
  
  /// Called when a proxy is added to the wm
  virtual void bindingUnionAdded(const cast::cdl::WorkingMemoryChange & _wmc) = 0;
  /// Called when a proxy is overwritten in the wm
  virtual void bindingUnionUpdated(const cast::cdl::WorkingMemoryChange & _wmc) = 0;
  /// Called when a proxy is deleted from the wm
  virtual void bindingUnionDeleted(const cast::cdl::WorkingMemoryChange & _wmc) = 0;

protected:

  AbstractActiveBindingWMReader(const std::string &_id);
  virtual ~AbstractActiveBindingWMReader() { };

  /// defines the ID of binding SA, must be set via configure
  std::string m_bindingSA;

public:
  
  void configure(std::map<std::string,std::string> & _config);
  
};

} // namespace Binding

#endif //  BINDING_ABSTRACT_ACTIVE_BINDING_WM_READER_H_
