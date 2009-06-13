#ifndef BINDING_ABSTRACT_FEATURE_GENERATPOR_H_ 
#define BINDING_ABSTRACT_FEATURE_GENERATPOR_H_ 
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <binding/abstr/AbstractMonitor.hpp>
#include <binding/idl/BindingQueries.hh>
#include <binding/utils/LocalClasses.hpp>
#include <cast/architecture/WorkingMemoryChangeReceiver.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/cdl/CAST.hh>
#include <boost/mefn.hpp>

namespace Binding {


/// Contains the datatypes needed to store binding WM locally
  class AbstractFeatureGenerator :
    public AbstractMonitor {
    
  private:
    bool filterUp;
    void handleGenerationCommand(const cast::cdl::WorkingMemoryChange & _wmc);


  protected:
    AbstractFeatureGenerator(const std::string & _id);
    virtual ~AbstractFeatureGenerator() { };


    ///Register the capability to generate a feature. This writes to wm,
    ///so should be called in the runComponent method or later. The
    ///input receiver is used in the same way to a change receiver, so
    ///should be treated the same (i.e. not free'd).
    template <class FeatureT, class T>
    void
    registerFeatureGenerator(void (T::*pmf)(ProxyPtr)) {    
      
      //register the filter for receiving generation commands
      if(!filterUp) {
	addChangeFilter(cast::createLocalTypeFilter<BindingQueries::FeatureGenerationCommand>(cast::cdl::ADD),
			new cast::MemberFunctionChangeReceiver<AbstractFeatureGenerator>(this,
												     &AbstractFeatureGenerator::handleGenerationCommand));
	filterUp = true;
      }

    //this is the type of feature we can add
    std::string featureType(cast::typeName<FeatureT>());

    //now generate the registration struct and add it
    BindingQueries::FeatureGenerationRegistration *fr = new BindingQueries::FeatureGenerationRegistration();
    fr->type = CORBA::string_dup(featureType.c_str());
    fr->subarchitecture = CORBA::string_dup(subarchitectureID().c_str());
    
    addToWorkingMemory(newDataID(), 
		       getBindingSA(), 
		       fr, cast::cdl::BLOCKING);

    log("send registration: %s", featureType.c_str());
  }
  
  
  ///State that feature generation is complete
  template <class FeatureT>
  void generationComplete(ProxyPtr _proxy, BindingData::TriBool _succeeded) {

  }
  
  public:
};


} // namespace Binding


#endif //  BINDING_ABSTRACT_FEATURE_GENERATPOR_H_
