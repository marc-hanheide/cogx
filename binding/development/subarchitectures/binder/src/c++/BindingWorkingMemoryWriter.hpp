#ifndef BINDING_WORKING_MEMORY_WRITER_HPP
#define BINDING_WORKING_MEMORY_WRITER_HPP

#include <cast/architecture/ManagedComponent.hpp>
#include <autogen/BinderEssentials.hpp>

namespace binder {

  /**
   * Abstract class for structuring and inserting proxies into the binder
   * working memory
   * 
   * Nick: Return values for the add* methods have been removed to
   * reduce potential ambiguity.
   *
   * @author Pierre Lison, Nick Hawes
   * @version 02/09/2009
   */
  class BindingWorkingMemoryWriter : 
    public cast::ManagedComponent {
    
    /**
     * Constructor
     */
    BindingWorkingMemoryWriter();
    
    /**
     * Destructor
     */
    virtual ~BindingWorkingMemoryWriter();
    

  protected:

    /** Create a new proxy given the ID of the originating subarchitecture,
     * and the probability of the proxy itself
     * (the list of features is defined to be empty)
     * 
     * @param subarchId string for the ID of the subarchitecture
     * @param probExists probability value for the proxy
     * @return a new proxy
     */
    autogen::core::ProxyPtr createNewProxy (const std::string & subarchId, 
					    float probExists) {
      
      autogen::core::ProxyPtr newProxy = new autogen::core::Proxy();
      
      newProxy->entityID = newDataID();
      newProxy->subarchId = subarchId;
      newProxy->probExists = probExists;
      
      return newProxy;
    }
    
    
    /**
     * Create a new proxy given the ID of the originating subarchitecture,
     * the probability of the proxy, and a list of features
     *  
     * @param subarchId string for the ID of the subarchitecture
     * @param probExists
     * @param features
     * @return
     */
    autogen::core::ProxyPtr createNewProxy (const std::string & subarchId, 
					    float probExists, 
					    const autogen::core::FeaturesList & features) {
      
      autogen::core::ProxyPtr newProxy = createNewProxy(subarchId, probExists);      
      newProxy->features = features;

      return newProxy;
    }
    
    
    /**
     * Add a new feature to the proxy (and regenerate the probability
     * distribution, given this new information)
     * 
     * @param proxy the proxy
     * @param feat the feature to add
     */
    void addFeatureToProxy(autogen::core::ProxyPtr proxy, 
			   autogen::core::FeaturePtr feat) {      
      proxy->features.push_back(feat);
    }
    
    
    
    /**
     * Create a new StringValue given a string and a probability
     * 
     * @param val the string
     * @param prob the probability value
     * @return the StringValue
     */
    autogen::featvalues::StringValuePtr createStringValue(const std::string & val, 
							  float prob) {
      autogen::featvalues::StringValuePtr stringVal = new autogen::featvalues::StringValue();
      stringVal->val = val;
      stringVal->independentProb = prob;
      return stringVal;
    }
    
    /** 
     * Create a new feature, without feature values
     * @param featlabel the feature label
     * @return the new feature
     */    
    autogen::core::FeaturePtr createFeature(const std::string & featlabel) {
      autogen::core::FeaturePtr feat = new autogen::core::Feature();
      feat->featlabel = featlabel;
      return feat;
    }

    /**
     * Create a new feature with a unique feature value
     * @param featlabel the feature label
     * @param featvalue the feature value
     * @return the new feature
     */
    
    autogen::core::FeaturePtr createFeatureWithUniqueFeatureValue 
    (const std::string & featlabel, autogen::core::FeatureValuePtr featvalue) {
      
      autogen::core::FeaturePtr feat = createFeature(featlabel);
      feat->alternativeValues.push_back(featvalue);      
      return feat;
    }
    
    
    /** 
     * Add a new feature value to an existing feature
     * @param feat the feature
     * @param featval the feature value
     */    
    void addFeatureValueToFeature(autogen::core::FeaturePtr feat, 
				  autogen::core::FeatureValuePtr featval) {      
      feat->alternativeValues.push_back(featval);            
    }
    
    /** 
     * Create a new feature containing several alternative feature values
     * @param featlabel the feature label
     * @param featvalues the array of feature values
     * @return the feature
     */    
    autogen::core::FeaturePtr createFeatureWithAlternativeFeatureValues 
    (const std::string & featlabel, const autogen::core::FeatureValues & featvalues) {
      
      autogen::core::FeaturePtr feat = createFeature(featlabel);
      feat->alternativeValues = featvalues;      
      return feat;
    }
    
    
    /** 
     * Insert the proxy in the binder working memory 
     * @param proxy the proxy
     */    
    void addProxyToWM(autogen::core::ProxyPtr proxy) {

      try {
	addToWorkingMemory(proxy->entityID, proxy);
	log("new Proxy succesfully added to the binder working memory");	
      }
      catch (cast::CASTException &e) {
	println("EXCEPTION in BindingWorkingMemoryWriter::addProxyToWM: %s", e.message.c_str());
      }
    }
    
    
    /**
     * Overwrite an existing proxy with a new one
     * (the new proxy needs to have the same entityID has the existing one)
     * 
     * @param proxy the new proxy
     */
    
    void overwriteProxyInWM(autogen::core::ProxyPtr proxy) {

      try {
	overwriteWorkingMemory(proxy->entityID, proxy);
	log("existing Proxy succesfully modified in the binder working memory");
	
      }
      catch (cast::CASTException &e) {
	println("EXCEPTION in BindingWorkingMemoryWriter::overwriteProxyInWM: %s", e.message.c_str());
      }
    }
    

    /**
     * Delete an existing proxy
     * @param proxy the proxy to delete
     */
    
    void deleteEntityInWM(autogen::core::ProxyPtr proxy) {

      try {
	deleteFromWorkingMemory(proxy->entityID);
	log("existing Proxy succesfully modified in the binder working memory");
	
      }
      catch (cast::CASTException &e) {
	println("EXCEPTION in BindingWorkingMemoryWriter::deleteEntityInWM: %s", e.message.c_str());
      }
    }

  };
  
}

#endif
