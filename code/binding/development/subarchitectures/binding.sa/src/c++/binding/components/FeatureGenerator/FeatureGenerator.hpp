#ifndef BINDING_FEATURE_GENERATOR_H_
#define BINDING_FEATURE_GENERATOR_H_

#include <balt/core/StringMap.hpp>
#include "binding/abstr/AbstractBinder.hpp"
#include <binding/utils/GraphLoader.hpp>
#include <binding/idl/BindingQueries.hh>


#include <ext/hash_set>


namespace Binding {

/// Calculates scores between proxies and instances
class FeatureGenerator: public AbstractBinder {


  typedef __gnu_cxx::hash_set<std::string> StringSet;
  
  typedef cast::StringMap<StringSet>::map StringSetMap;

public:
  FeatureGenerator(const std::string &_id);
  virtual ~FeatureGenerator();
  
  virtual void runComponent(){};  
  
  virtual void start();
protected:

  virtual void taskAdopted(const std::string &_taskID){}
  virtual void taskRejected(const std::string &_taskID){}
  virtual void configure(std::map<std::string,std::string>& _config){AbstractBinder::configure(_config);}


private:
  void registerFeature(const cast::cdl::WorkingMemoryChange & _wmc);
  void newFeatureRequest(const cast::cdl::WorkingMemoryChange & _wmc);
  void generationComplete(const cast::cdl::WorkingMemoryChange & _wmc);

  void answerRequest(const cast::cdl::WorkingMemoryAddress & _wma,
		     boost::shared_ptr<const cast::CASTData<BindingQueries::FeatureRequest> > _request,
		     const BindingData::TriBool & _successful);
  

  StringSetMap feature2sa;

  //handler for manipulating proxies and unions
  BindingGraphHandler handler;

};

} // namespace Binding

#endif // BINDING_DISAMBIGUATION_GENERATOR_H_
