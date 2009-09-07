#include "BindingWorkingMemoryWriter.hpp"


namespace binder {

  using namespace autogen::core;
  using namespace autogen::featvalues;
  using namespace cast;
   
  ProxyPtr BindingWorkingMemoryWriter::createNewProxy (const std::string & subarchId, 
						       float probExists) {
    
    ProxyPtr newProxy = new Proxy();
    
    newProxy->entityID = newDataID();
    newProxy->subarchId = subarchId;
    newProxy->probExists = probExists;
      
    return newProxy;
  }
    
    
  ProxyPtr BindingWorkingMemoryWriter::createNewProxy (const std::string & subarchId, 
								      float probExists, 
								      const FeaturesList & features) {
      
    ProxyPtr newProxy = createNewProxy(subarchId, probExists);      
    newProxy->features = features;

    return newProxy;
  }
    
  void BindingWorkingMemoryWriter::addFeatureToProxy(ProxyPtr proxy, 
						     FeaturePtr feat) {      
    proxy->features.push_back(feat);
  }
    
    
  StringValuePtr BindingWorkingMemoryWriter::createStringValue(const std::string & val, 
										    float prob) {
    StringValuePtr stringVal = new StringValue();
    stringVal->val = val;
    stringVal->independentProb = prob;
    return stringVal;
  }
    
  FeaturePtr BindingWorkingMemoryWriter::createFeature(const std::string & featlabel) {
    FeaturePtr feat = new Feature();
    feat->featlabel = featlabel;
    return feat;
  }

    
  FeaturePtr BindingWorkingMemoryWriter::createFeatureWithUniqueFeatureValue 
  (const std::string & featlabel, FeatureValuePtr featvalue) {
      
    FeaturePtr feat = createFeature(featlabel);
    feat->alternativeValues.push_back(featvalue);      
    return feat;
  }
    
    
  void BindingWorkingMemoryWriter::addFeatureValueToFeature(FeaturePtr feat, 
							    FeatureValuePtr featval) {      
    feat->alternativeValues.push_back(featval);            
  }
    
  FeaturePtr BindingWorkingMemoryWriter::createFeatureWithAlternativeFeatureValues 
  (const std::string & featlabel, const FeatureValues & featvalues) {
      
    FeaturePtr feat = createFeature(featlabel);
    feat->alternativeValues = featvalues;      
    return feat;
  }
    
  void BindingWorkingMemoryWriter::addProxyToWM(ProxyPtr proxy) {

    try {
      addToWorkingMemory(proxy->entityID, proxy);
      log("new Proxy succesfully added to the binder working memory");	
    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::addProxyToWM: %s", e.message.c_str());
    }
  }
    
    
    
  void BindingWorkingMemoryWriter::overwriteProxyInWM(ProxyPtr proxy) {

    try {
      overwriteWorkingMemory(proxy->entityID, proxy);
      log("existing Proxy succesfully modified in the binder working memory");
	
    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::overwriteProxyInWM: %s", e.message.c_str());
    }
  }
    

    
  void BindingWorkingMemoryWriter::deleteEntityInWM(ProxyPtr proxy) {

    try {
      deleteFromWorkingMemory(proxy->entityID);
      log("existing Proxy succesfully modified in the binder working memory");
	
    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::deleteEntityInWM: %s", e.message.c_str());
    }
  }



}
