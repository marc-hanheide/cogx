#ifndef COPYCAT_MONITOR_H_ 
#define COPYCAT_MONITOR_H_

#include <binding/abstr/AbstractMonitor.hpp>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <ext/hash_set>

namespace Binding {

//typedef std::set<std::string> stringSet;
typedef __gnu_cxx::hash_set<std::string> StringSet;

/// a monitor that copies all features and proxies from the binder
/// into another binder (which \p CopyCatBindingMonitor should be a
/// component in)
class CopyCatBindingMonitor : 
    public AbstractMonitor,
    public AbstractBindingWMRepresenter
{
  /// maps from the IDs of the local binder's proxies to the corresponding IDs of the target binder
  cast::StringMap<std::string>::map m_proxyIDMap;
  // proxies that have actually been created on the virtual binder
  StringSet m_createdProxyIDs;
  /// features that are excluded when copying proxies
  std::set<std::string> m_excludedFeatures;

public:
  CopyCatBindingMonitor(const std::string &_id);
  virtual ~CopyCatBindingMonitor(){};
  
  void start();
  void configure(std::map<std::string,std::string> & _config);
  
  /// causes the proxies to be copied to the target binder
  void bindTheseProxiesAdded(const cast::cdl::WorkingMemoryChange& _wmc);
  /// causes the proxies to be deleted from the target binder
  void proxyDeleted(const cast::cdl::WorkingMemoryChange& _wmc);
  
protected:
  void taskAdopted(const std::string &_taskID){}
  void taskRejected(const std::string &_taskID){}
  void runComponent(){};
  void stop() {cast::CASTComponent::stop();}
}; 

} // namespace Binding

#endif //COPYCAT_MONITOR_H_

