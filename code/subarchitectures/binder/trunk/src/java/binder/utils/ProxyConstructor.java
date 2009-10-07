package binder.utils;


import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
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
		
		WorkingMemoryPointer origin = new WorkingMemoryPointer();
		origin.address = new WorkingMemoryAddress(localDataId, subarchId);
		origin.type = localDataType;
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

		Proxy newProxy = new Proxy();

		newProxy.entityID = entityID;
		newProxy.origin = origin;
		newProxy.probExists = probExists;
		newProxy.features = new Feature[0];

		return newProxy;
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

		PhantomProxy newProxy = new PhantomProxy();

		newProxy.entityID = entityID;
		newProxy.origin = origin;
		newProxy.probExists = probExists;
		newProxy.features = new Feature[0];

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

		RelationProxy newProxy = new RelationProxy();

		newProxy.entityID = entityID;
		newProxy.origin = origin;
		newProxy.probExists = probExists;
		newProxy.features = new Feature[0];

		newProxy.source = new Feature();
		newProxy.source.featlabel = "source";
		newProxy.source.alternativeValues = sources;

		newProxy.target = new Feature();
		newProxy.target.featlabel = "target";
		newProxy.target.alternativeValues = targets;

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

	public static StringValue createStringValue(String val, float prob, CASTTime timestamp) {
		StringValue stringVal = new StringValue();
		stringVal.val = val;
		stringVal.independentProb = prob;
		stringVal.timeStamp = timestamp;
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

	public static AddressValue createAddressValue(String address, float prob, CASTTime timestamp) {
		AddressValue addressVal = new AddressValue();
		addressVal.val = address;
		addressVal.independentProb = prob;
		addressVal.timeStamp = timestamp;
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

	public static FloatValue createFloatValue(float floatv, float prob, CASTTime timestamp) {
		FloatValue floatVal = new FloatValue();
		floatVal.val = floatv;
		floatVal.independentProb = prob;
		floatVal.timeStamp = timestamp;
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

	public static IntegerValue createIntegerValue(int integer, float prob, CASTTime timestamp) {
		IntegerValue integerVal = new IntegerValue();
		integerVal.val = integer;
		integerVal.independentProb = prob;
		integerVal.timeStamp = timestamp;
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

	public static BooleanValue createBooleanValue(boolean val, float prob, CASTTime timestamp) {
		BooleanValue boolVal = new BooleanValue();
		boolVal.val = val;
		boolVal.independentProb = prob;
		boolVal.timeStamp = timestamp;
		return boolVal;
	}
	
	
	

	/**
	 * Create a new UnknownValue given a probability
	 * 
	 * @param prob
	 *            the probability value
	 * @return the BooleanValue
	 */

	public static UnknownValue createUnknownValue(float prob, CASTTime timestamp) {
		UnknownValue unknownVal = new UnknownValue();
		unknownVal.independentProb = prob;
		unknownVal.timeStamp = timestamp;
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
		Feature feat = new Feature();
		feat.featlabel = featlabel;
		feat.alternativeValues = new FeatureValue[0];
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
