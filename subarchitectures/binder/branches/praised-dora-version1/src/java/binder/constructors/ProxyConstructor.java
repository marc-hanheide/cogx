package binder.constructors;


import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import cast.core.logging.ComponentLogger;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.FloatValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.featvalues.UnknownValue;
import binder.autogen.specialentities.PhantomProxy;
import binder.autogen.specialentities.RelationProxy;

public class ProxyConstructor {


	private static Logger logger = ComponentLogger.getLogger(ProxyConstructor.class);


	/**
	 * Construct an WorkingMemoryPointer object (information about the proxy origin:
	 * subarchitecture identifier, local data ID in subarchitecture, and data
	 * type)
	 * 
	 * @param subarchId
	 *            subarchitecture identifier
	 * @param localDataId
	 *            identifier of the data in local subarchitecture
	 * @param localDataType
	 *            type of the data in local subarchitecture
	 * @return a new OriginInfo object
	 */
	public static WorkingMemoryPointer createWorkingMemoryPointer (String subarchId, String localDataId,
			String localDataType) {
		
		WorkingMemoryAddress address = new WorkingMemoryAddress(localDataId, subarchId);
		String type = localDataType;
		WorkingMemoryPointer origin = new WorkingMemoryPointer(address, type);

		return origin;
	}
	
	/**
	 * Create a new proxy given the pointer to the originating object and
	 * the probability of the proxy existence itself (the list of features 
	 * is defined to be empty)
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            probability value for the proxy
	 * @return a new proxy
	 */
	public static Proxy createNewProxy(WorkingMemoryPointer origin, String entityID, float probExists) {
	
		ProbabilityDistribution emptyDistrib = new ProbabilityDistribution();	
		Proxy newProxy = new Proxy(entityID, probExists, new CASTTime(0,0), new Feature[0], emptyDistrib, origin);

		return newProxy;
	}
	
	
	/**
	 * Set the timestamp
	 * 
	 * @param proxy
	 * @param timestamp
	 */
	public static void setTimeStamp (Proxy proxy, CASTTime timestamp) {
		proxy.timeStamp = timestamp;
	}
	
	
	
	/**
	 * Set the timestamp
	 * 
	 * @param proxy
	 * @param timestamp
	 */
	public static void setTimeStamp (FeatureValue featval, CASTTime timestamp) {
		featval.timeStamp = timestamp;
	}
	
	
	/**
	 * Create a new proxy given the pointer to the originating object, the
	 * probability of the proxy, and a list of features
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            the probability of the proxy
	 * @param features
	 *            the features
	 * @return the created proxy
	 */
	public static Proxy createNewProxy(WorkingMemoryPointer origin, String entityID, float probExists,
			Feature[] features) {

		Proxy newProxy = createNewProxy(origin, entityID, probExists);

		newProxy.features = features;

		return newProxy;
	}
	
	/**
	 * Create a new phantom proxy, given the origin info and the existence
	 * probability
	 * 
	 * @param origin
	 *            origin information
	 * @param probExists
	 *            the probability of the proxy
	 * @return the new phantom
	 */
	public static PhantomProxy createNewPhantomProxy
		(WorkingMemoryPointer origin, String entityID, float probExists) {

		ProbabilityDistribution emptyDistrib = new ProbabilityDistribution();	
		PhantomProxy newProxy = new PhantomProxy(entityID, probExists, new CASTTime(0,0), new Feature[0], emptyDistrib, origin);

		return newProxy;
	}
 
	/**
	 * Create a new phantom proxy, given the origin info and the existence
	 * probability
	 * 
	 * @param origin
	 *            origin information
	 * @param probExists
	 *            the probability of the proxy
	 * @return the new phantom
	 */
	public static PhantomProxy createNewPhantomProxy
		(WorkingMemoryPointer origin, String entityID, float probExists, Feature[] features) {

		PhantomProxy newProxy = createNewPhantomProxy(origin, entityID, probExists);
		newProxy.features = features;

		return newProxy;
	}

	/**
	 * Add a new feature to the proxy (and regenerate the probability
	 * distribution, given this new information)
	 * 
	 * @param proxy  
	 *            the proxy
	 * @param feat
	 *            the feature to add
	 */

	public static void addFeatureToProxy(Proxy proxy, Feature feat) {

		Feature[] newFeatures;
		
		if (feat.featlabel == null) {
			System.out.println("WARNING: feature has no specified feature label, cannot insert feature");
		}
		else if (feat.alternativeValues == null) {
			System.out.println("WARNING: feature has no specified feature values, cannot insert feature");
		}
		else if (feat.alternativeValues.length == 0) {
			System.out.println("WARNING: feature has no specified feature values, cannot insert feature");
		}
		
		if (proxy.features != null) {
			newFeatures = new Feature[proxy.features.length + 1];
			for (int i = 0; i < proxy.features.length; i++) {
				newFeatures[i] = proxy.features[i];
			}
			newFeatures[proxy.features.length] = feat;
		} else {
			newFeatures = new Feature[1];
			newFeatures[0] = feat;
		}
		
		proxy.features = newFeatures;

	}

	

	/**
	 * Create a new relation proxy given the pointer to the originating object,
	 * the probability of the proxy, and the source and target proxies
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            the probability of the proxy
	 * @param sources
	 *            the addresses of the source proxies
	 * @param targets
	 *            the addresses of the target proxies
	 * @return the new relation proxy
	 */
	public static RelationProxy createNewRelationProxy(WorkingMemoryPointer origin,
			String entityID, float probExists, AddressValue[] sources, AddressValue[] targets) {


		Feature source = new Feature("source", sources);

		Feature target = new Feature("target", targets);

		ProbabilityDistribution emptyDistrib = new ProbabilityDistribution();	

		RelationProxy newProxy = new RelationProxy
			(entityID, probExists, new CASTTime(0,0), new Feature[0], emptyDistrib, origin,source, target);

		return newProxy;
	}

	/**
	 * Create a new relation proxy given the pointer to the originating object,
	 * the probability of the proxy, the list of features for
	 * the relation, and the source and target proxies
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            the probability of the proxy
	 * @param features
	 *            the features
	 * @param sources
	 *            the addresses of the source proxies
	 * @param targets
	 *            the addresses of the target proxies
	 * @return the new relation proxy
	 */
	public static RelationProxy createNewRelationProxy(WorkingMemoryPointer origin,
			String entityID, float probExists, Feature[] features, AddressValue[] sources,
			AddressValue[] targets) {

		RelationProxy newProxy = createNewRelationProxy(origin, entityID, probExists,
				sources, targets);

		newProxy.features = features;

		return newProxy;
	}



	/**
	 * Create a new StringValue given a string and a probability
	 * 
	 * @param val
	 *            the string
	 * @param prob
	 *            the probability value
	 * @return the StringValue
	 */

	public static StringValue createStringValue(String val, float prob) {
		StringValue stringVal = new StringValue(prob, new CASTTime(0,0), val);
		return stringVal;
	}

	/**
	 * Create a new AddressValue given a string and a probability
	 * 
	 * @param address
	 *            the address (as a string)
	 * @param prob
	 *            the probability value
	 * @return the AddressValue
	 */

	public static AddressValue createAddressValue(String address, float prob) {
		AddressValue addressVal = new AddressValue(prob, new CASTTime(0,0), address);
		return addressVal;
	}

	/**
	 * Create a new AddressValue given a string and a probability
	 * 
	 * @param integer
	 *            the integer
	 * @param prob
	 *            the probability value
	 * @return the IntegerValue
	 */

	public static FloatValue createFloatValue(float floatv, float prob) {
		FloatValue floatVal = new FloatValue(prob, new CASTTime(0,0), floatv);
		return floatVal;
	} 
	
	
	/**
	 * Create a new AddressValue given a string and a probability
	 * 
	 * @param integer
	 *            the integer
	 * @param prob
	 *            the probability value
	 * @return the IntegerValue
	 */

	public static IntegerValue createIntegerValue(int integer, float prob) {
		IntegerValue integerVal = new IntegerValue(prob, new CASTTime(0,0), integer);
		return integerVal;
	}

	/**
	 * Create a new BooleanValue given a boolean and a probability
	 * 
	 * @param val
	 *            the boolean
	 * @param prob
	 *            the probability value
	 * @return the BooleanValue
	 */

	public static BooleanValue createBooleanValue(boolean val, float prob) {
		BooleanValue boolVal = new BooleanValue(prob, new CASTTime(0,0), val);
		return boolVal;
	}
	
	
	

	/**
	 * Create a new UnknownValue given a probability
	 * 
	 * @param prob
	 *            the probability value
	 * @return the BooleanValue
	 */

	public static UnknownValue createUnknownValue(float prob) {
		UnknownValue unknownVal = new UnknownValue(prob, new CASTTime(0,0));
		return unknownVal;
	}


	/**
	 * Create a new feature, without feature values
	 * 
	 * @param featlabel
	 *            the feature label
	 * @return the new feature
	 */

	public static Feature createFeature(String featlabel) {
		Feature feat = new Feature(featlabel, new FeatureValue[0]);
		return feat;
	} 

	/**
	 * Create a new feature with a unique feature value
	 * 
	 * @param featlabel
	 *            the feature label
	 * @param featvalue
	 *            the feature value
	 * @return the new feature
	 */

	public static Feature createFeatureWithUniqueFeatureValue(String featlabel,
			FeatureValue featvalue) {

		Feature feat = createFeature(featlabel);
		feat.alternativeValues = new FeatureValue[1];
		feat.alternativeValues[0] = featvalue;

		return feat;
	}

	/**
	 * Add a new feature value to an existing feature
	 * 
	 * @param feat
	 *            the feature
	 * @param featval
	 *            the feature value
	 * @return the feature
	 */

	public static Feature addFeatureValueToFeature(Feature feat, FeatureValue featval) {

		if (feat.alternativeValues != null) {
		FeatureValue[] featvals = new FeatureValue[feat.alternativeValues.length +1];

		for (int i = 0; i < feat.alternativeValues.length; i++) {
			featvals[i] = feat.alternativeValues[i];
		}
		featvals[feat.alternativeValues.length] = featval;
		feat.alternativeValues = featvals;
		}
		else {
			feat.alternativeValues = new FeatureValue[1];
			feat.alternativeValues[0] = featval;
		}
		return feat;
	}

	/**
	 * Create a new feature containing several alternative feature values
	 * 
	 * @param featlabel
	 *            the feature label
	 * @param featvalues
	 *            the array of feature values
	 * @return the feature
	 */

	public static Feature createFeatureWithAlternativeFeatureValues(String featlabel,
			FeatureValue[] featvalues) {

		Feature feat = createFeature(featlabel);
		feat.alternativeValues = featvalues;

		return feat;
	}

}
