#ifndef BINDING_DUMMY_FEATURE_GENERATOR_H_
#define BINDING_DUMMY_FEATURE_GENERATOR_H_

#include <binding/abstr/AbstractFeatureGenerator.hpp>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <binding/abstr/AbstractMonitor.hpp>
#include <cast/architecture/PrivilegedManagedProcess.hpp>


namespace Binding {

  /// Dummy generator for testing
  class DummyGenerator: 
    public AbstractFeatureGenerator   {

  public:
    DummyGenerator(const std::string &_id);
    virtual ~DummyGenerator();
    
    virtual void runComponent();  
    virtual void configure(std::map<std::string,std::string> & _config);
    
  protected:
    
    virtual void taskAdopted(const std::string &_taskID){}
    virtual void taskRejected(const std::string &_taskID){}
    
    template <class FeatureT>
    BindingQueries::FeatureRequest * 
    generateFeatureRequest(const std::string _proxyID) {
      BindingQueries::FeatureRequest * req = new BindingQueries::FeatureRequest();
      req->m_type = CORBA::string_dup(cast::typeName<FeatureT>().c_str());
      req->m_subarchitecture = CORBA::string_dup(subarchitectureID().c_str());
      req->m_proxyID = CORBA::string_dup(_proxyID.c_str());
      req->m_processed = false;
      req->m_successful = cast::cdl::triIndeterminate;
      return req;
    }

  private:

    void
    generateColour(ProxyPtr _proxy) {
      println("generateColour");
      generationComplete<BindingFeatures::Colour>(_proxy,
						   cast::cdl::triTrue);
    };

    void
    requestAnswered(const cast::cdl::WorkingMemoryChange & _wmc);

    bool m_requester;
 
  };

} // namespace Binding

#endif // BINDING_DISAMBIGUATION_GENERATOR_H_
