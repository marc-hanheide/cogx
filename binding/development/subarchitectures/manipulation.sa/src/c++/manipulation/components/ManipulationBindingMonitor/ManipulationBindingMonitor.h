#ifndef MANIPULATION_BINDING_MONITOR_H
#define MANIPULATION_BINDING_MONITOR_H

#include <map>
#include <string>
#include <cast/architecture/ManagedProcess.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/BindingException.hpp>

#include <binding/FeatureProperties.hpp>
#include <binding/AnyFeature.hpp>

#include <binding/abstr/AbstractMonitor.hpp>

namespace Binding
{

class ManipulationBindingMonitor : public AbstractMonitor
{
private:
  map<string,string> m_sourceProxyMapping;

  void locationAdded(const cdl::WorkingMemoryChange& _wmc);
  void locationUpdated(const cdl::WorkingMemoryChange& _wmc);
  void locationDeleted(const cdl::WorkingMemoryChange& _wmc);

protected:
  void taskAdopted(const string &_taskID) {}
  void taskRejected(const string &_taskID) {}
  void runComponent() {}

public:
  ManipulationBindingMonitor(const string _id);
  /// overridden start method used to set filters
  virtual void start();
};

}

#endif

