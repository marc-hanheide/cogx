package binder.abstr;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbDistribUtils;
import cast.DoesNotExistOnWMException;
import cast.architecture.ManagedComponent;

/**
 * Abstract class for structuring and inserting proxies into the binder
 * working memory
 * 
 * @author Pierre Lison
 * @version 31/08/2009
 */
public abstract class BindingWorkingMemoryWriter extends ManagedComponent {


	/** Create a new proxy given the ID of the originating subarchitecture,
	 * and the probability of the proxy itself
	 * (the list of features is defined to be empty)
	 * 
	 * @param subarchId string for the ID of the subarchitecture
	 * @param probExists probability value for the proxy
	 * @return a new proxy
	 */
	public Proxy createNewProxy (String subarchId, float probExists) {
		
		Proxy newProxy = new Proxy();
		
		newProxy.entityID = newDataID();
		newProxy.subarchId = subarchId;
		newProxy.probExists = probExists;
		newProxy.features = new Feature[0];
	
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
	public Proxy createNewProxy (String subarchId, float probExists, Feature[] features) {
		
		Proxy newProxy = createNewProxy(subarchId, probExists);
		
		newProxy.features = features;
	
		newProxy.features = ProbDistribUtils.addIndeterminateFeatureValues(newProxy.features);
		newProxy.distribution = ProbDistribUtils.generateProbabilityDistribution(newProxy);

		return newProxy;
	}
	
	
	/**
	 * Add a new feature to the proxy (and regenerate the probability distribution, 
	 * given this new information)
	 * 
	 * @param proxy the proxy
	 * @param feat the feature to add
	 * @return the proxy
	 */
	
	public Proxy addFeatureToProxy (Proxy proxy, Feature feat) {
		
		Feature[] newFeatures;
		if (proxy.features != null) {
		newFeatures = new Feature[proxy.features.length + 1] ;
		for (int i = 0 ; i < proxy.features.length ; i++) {
			newFeatures[i] = proxy.features[i];
		}
		newFeatures[proxy.features.length] = feat;
		}
		else {
			newFeatures = new Feature [1];
			newFeatures[0] = feat;
		}
		
		proxy.features = newFeatures;
		
		proxy.features = ProbDistribUtils.addIndeterminateFeatureValues(proxy.features);
		proxy.distribution = ProbDistribUtils.generateProbabilityDistribution(proxy);

		return proxy;
	}
	
	
	
	/**
	 * Create a new StringValue given a string and a probability
	 * 
	 * @param val the string
	 * @param prob the probability value
	 * @return the StringValue
	 */
	
	public StringValue createStringValue (String val, float prob) {
		StringValue stringVal = new StringValue();
		stringVal.val = val;
		stringVal.independentProb = prob;
		return stringVal;
	}
	
	/** 
	 * Create a new feature, without feature values
	 * @param featlabel the feature label
	 * @return the new feature
	 */
	
	public Feature createFeature (String featlabel) {
		Feature feat = new Feature();
		feat.featlabel = featlabel;
		return feat;
	}
	
	/**
	 * Create a new feature with a unique feature value
	 * @param featlabel the feature label
	 * @param featvalue the feature value
	 * @return the new feature
	 */
	
	public Feature createFeatureWithUniqueFeatureValue 
		(String featlabel, FeatureValue featvalue) {
		
		Feature feat = createFeature(featlabel);
		feat.alternativeValues = new FeatureValue[1];
		feat.alternativeValues[0] = featvalue;
		
		return feat;
	}
	
	
	/** 
	 * Add a new feature value to an existing feature
	 * @param feat the feature
	 * @param featval the feature value
	 * @return the feature
	 */
	
	public Feature addFeatureValueToFeature (Feature feat, FeatureValue featval) {
		
		FeatureValue[] featvals = new FeatureValue[feat.alternativeValues.length +1];
		
		for (int i = 0 ; i < feat.alternativeValues.length ; i++) {
			featvals[i] = feat.alternativeValues[i];
		}
		featvals[feat.alternativeValues.length] = featval;
		feat.alternativeValues = featvals;
		
		return feat;
	}
	
	/** 
	 * Create a new feature containing several alternative feature values
	 * @param featlabel the feature label
	 * @param featvalues the array of feature values
	 * @return the feature
	 */
	
	public Feature createFeatureWithAlternativeFeatureValues 
	(String featlabel, FeatureValue[] featvalues) {
		
		Feature feat = createFeature(featlabel);
		feat.alternativeValues = featvalues;
		
		return feat;
	}
	
	
	/** 
	 * Insert the proxy in the binder working memory 
	 * @param proxy the proxy
	 */
	
	protected void addProxyToWM(Proxy proxy) {

		try {
		addToWorkingMemory(proxy.entityID, proxy);
		log("new Proxy succesfully added to the binder working memory");
		
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	/**
	 * Overwrite an existing proxy with a new one
	 * (the new proxy needs to have the same entityID has the existing one)
	 * 
	 * @param proxy the new proxy
	 */
	
	protected void overwriteProxyInWM(Proxy proxy) {

		try {
		overwriteWorkingMemory(proxy.entityID, proxy);
		log("existing Proxy succesfully modified in the binder working memory");
		
		}
		catch (DoesNotExistOnWMException e) {
			log("Sorry, the proxy does not exist in the binder working memory");
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	

	/**
	 * Delete an existing proxy
	 * @param proxy the proxy to delete
	 */
	
	protected void deleteEntityInWM(Proxy proxy) {

		try {
		deleteFromWorkingMemory(proxy.entityID);
		log("existing Proxy succesfully modified in the binder working memory");
		
		}
		catch (DoesNotExistOnWMException e) {
			log("Sorry, the proxy does not exist in the binder working memory");
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
