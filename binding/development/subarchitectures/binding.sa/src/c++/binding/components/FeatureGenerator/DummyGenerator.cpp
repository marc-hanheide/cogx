#include "DummyGenerator.hpp"
#include <binding/idl/BindingFeatures.hh>

using namespace std;
using namespace boost;
using namespace cast;


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::DummyGenerator(_id);
  }
}

namespace Binding {

  using namespace BindingQueries;
  
  DummyGenerator::DummyGenerator(const string &_id) :         
    WorkingMemoryAttachedComponent(_id),
    AbstractFeatureGenerator(_id),
    requester(false)
  { 
  }
  

  DummyGenerator::~DummyGenerator() {
    
  }
  
  void DummyGenerator::configure(map<string,string> & _config) {
    AbstractMonitor::configure(_config);
    requester = (_config.find("--requester") != _config.end());
    setBindingSA("binding.sa");
  }

  void DummyGenerator::runComponent() {

    sourceID = subarchitectureID();

    setBindingSA("binding.sa");

    //some dummy generators
    registerFeatureGenerator<BindingFeatures::Colour>(&DummyGenerator::generateColour);

    if(!requester) {

      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup("thing");

      //add two thing proxies

      startNewBasicProxy();
      addFeatureToCurrentProxy(concept);
      addOtherSourceIDToCurrentProxy(sourceID, BindingFeaturesCommon::NEGATIVE);
      storeCurrentProxy();

      startNewBasicProxy();
      addFeatureToCurrentProxy(concept);
      addOtherSourceIDToCurrentProxy(sourceID, BindingFeaturesCommon::NEGATIVE);
      storeCurrentProxy();

      bindNewProxies();
    }
    else {
      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup("thing");

      //add two thing proxies

      startNewBasicProxy();
      addFeatureToCurrentProxy(concept);
      addOtherSourceIDToCurrentProxy(sourceID, BindingFeaturesCommon::NEGATIVE);
      BindingFeatures::Colour colour;
      colour.colour = CORBA::string_dup("red");
      addFeatureToCurrentProxy(colour);

      string myProxy(storeCurrentProxy());

      bindNewProxies();

      sleepProcess(5000);
      

      string reqID(newDataID());
      string reqSA(bindingSA());

      addChangeFilter(createAddressFilter(reqID,reqSA,cdl::OVERWRITE),
		      new MemberFunctionChangeReceiver<DummyGenerator>(this,
								       &DummyGenerator::requestAnswered));


      
      FeatureRequest *req = generateFeatureRequest<BindingFeatures::Colour>(myProxy);
      //FeatureRequest *req = generateFeatureRequest<BindingFeatures::Location>(myProxy);
      addToWorkingMemory(reqID,reqSA, req);
      log("requesting a colour feature");

    }



  }


  void
  DummyGenerator::requestAnswered(const cast::cdl::WorkingMemoryChange & _wmc) {
    log("DummyGenerator::requestAnswered");
  }



} // namespace Binding 

