#ifndef BINDING_AMBIGUITY_IDENTIFIER_H_
#define BINDING_AMBIGUITY_IDENTIFIER_H_

#include "binding/abstr/AbstractBinder.hpp"
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>


namespace Binding {

/// Calculates scores between proxies and instances
class AmbiguityIdentifier: public AbstractBinder {
public:
  AmbiguityIdentifier(const std::string &_id);
  virtual ~AmbiguityIdentifier();

  virtual void runComponent(){};  
  
  virtual void start();
  
  /// Triggered by a BestUnionsForProxy update (the creation is just
  /// empty). If the best list contains more than one union, the we
  /// have a disambiguation issue.
  void identifyDisambiguation(const cast::cdl::WorkingMemoryChange & _wmc);  
  /// deletes any disambiguation issues assoiated with the proxy
  void proxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc);
protected:

  virtual void taskAdopted(const std::string &_taskID){}
  virtual void taskRejected(const std::string &_taskID){}
  virtual void configure(std::map<std::string,std::string>& _config){AbstractBinder::configure(_config);}

private:
  std::set<std::string> _comparable_types(const BindingData::FeaturePointers& _ptrs) const;

};

} // namespace Binding

#endif // BINDING_DISAMBIGUATION_IDENTIFIER_H_
