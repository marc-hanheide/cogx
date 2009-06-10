#ifndef BINDING_WORKING_MEMORY_PROTOCOL_H_
#define BINDING_WORKING_MEMORY_PROTOCOL_H_

#include <cast/architecture/SubarchitectureWorkingMemoryProtocol.hpp>

#include "binding/idl/BindingFeatures.hh"
#include "binding/idl/BindingData.hh"

namespace Binding {

  typedef BindingFeatures::SourceData SourceData;

  class BindingWorkingMemoryProtocol :
    public cast::SubarchitectureWorkingMemoryProtocol {
   
  private:
    static const std::string SOURCE_DATA_PREFIX;
    
    template<class FeatureType> 
    static cast::WorkingMemoryPullQuery<FeatureType> *
    extractBindingFeature(const std::string &_query) {
      throw std::runtime_error("extractBindingFeature<FeatureType> not defined. ");
    }


  public:
    static cast::AbstractWorkingMemoryPullQuery * parseQuery(const std::string &_query);
       
  };

  template<>
  cast::WorkingMemoryPullQuery<BindingFeatures::SourceData> *
  BindingWorkingMemoryProtocol::extractBindingFeature<SourceData>(const std::string &_query);


}
#endif
