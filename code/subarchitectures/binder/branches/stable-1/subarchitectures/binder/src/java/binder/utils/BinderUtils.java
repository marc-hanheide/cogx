package binder.utils;

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.UnknownValue;
import binder.autogen.specialentities.RelationProxy;
import binder.autogen.specialentities.RelationUnion;

public class BinderUtils {

	public static RelationUnion convertIntoRelationUnion (Union bunion) {
		
		RelationUnion runion = new RelationUnion();
		runion.entityID = bunion.entityID;
		runion.features = bunion.features;
		runion.distribution = bunion.distribution;
		runion.includedProxies = bunion.includedProxies;
		runion.probExists = bunion.probExists;
		runion.timeStamp = bunion.timeStamp;
		
		return runion;
	}
	
	public static boolean induceRelationUnion(Vector<PerceivedEntity> entities) {

		Vector<Proxy> includedProxies = getProxies(entities);
		for (Enumeration<Proxy> e = includedProxies.elements() ; e.hasMoreElements() ;) {
			Proxy proxy = e.nextElement();
			if (! (proxy instanceof RelationProxy)) {
				return false;
			}
		}

		return true;
	}
	


	public static Feature[] addUnknownFeatureValues (Feature[] features) {

		for (int i = 0 ; i < features.length ; i++) {

			float totalProb = 0.0f;
			Vector<FeatureValue> values = new Vector<FeatureValue>();
			for (int j= 0 ; j < features[i].alternativeValues.length ; j++) {
				values.add(features[i].alternativeValues[j]);
				totalProb += features[i].alternativeValues[j].independentProb;
			}
			if (totalProb < 0.6f && features.length < 3) {
				features[i].alternativeValues = new FeatureValue[values.size() + 1];
				for (int j = 0 ; j < values.size(); j++ ) {
					features[i].alternativeValues[j] = values.elementAt(j);
				}
				features[i].alternativeValues[values.size()] = new UnknownValue((1.0f - totalProb));
			}
		}

		return features;
	}


	/**
	 * Get all the proxies included in a set of perceptual entities (in case the entity
	 * is a proxy, it is simply added, and in case it is an union, the set of all included
	 * proxies is added to the resulting set)
	 * @param includedEntities the set of perceptual entities
	 * @return the resulting set of proxies
	 */
	public static Vector<Proxy> getProxies (Vector<PerceivedEntity> includedEntities) {
		Vector<Proxy> includedProxies = new Vector<Proxy>();
		for (Enumeration<PerceivedEntity> en = includedEntities.elements() ; en.hasMoreElements() ;) {
			PerceivedEntity entity = en.nextElement();
			if (entity instanceof Proxy) {
				includedProxies.add((Proxy)entity);
			}
			else if (entity instanceof Union) {
				Union includedUnion = (Union)entity;
				for (int i = 0 ; i < includedUnion.includedProxies.length ; i++) {
					includedProxies.add(includedUnion.includedProxies[i]);
				}
			}
		}
		return includedProxies;
	}
	
	

	public static Proxy completeProxy (Proxy proxy, boolean addUnknowns) {
	// If necessary, add unknown values
	if (addUnknowns && !FeatureValueUtils.hasUnknownValues(proxy.features)) {
		proxy.features = 
			BinderUtils.addUnknownFeatureValues(proxy.features);
	}
	
	// if the probability distribution of the updated proxy is unavailable, regenerate it
	if (proxy.distribution == null) {
		proxy.distribution = 
			ProbabilityUtils.generateProbabilityDistribution(proxy);
	}

	return proxy;
}


	public static String getPrettyPrintProbabilityDistribution
	(DiscreteProbabilityDistribution distrib) {
	String text = "";

	if (distrib.assignments != null) {
	for (int i = 0; i < distrib.assignments.length; i++) {
		DiscreteProbabilityAssignment assignment = distrib.assignments[i];
		text += getPrettyPrintProbabilityAssignment(assignment) + "\n";
	}
	}
	return text;
}

public static String getPrettyPrintProbabilityAssignment
	(DiscreteProbabilityAssignment assignment) {

	String text = "P ( " ;
	for (int j = 0; j < assignment.featurepairs.length ; j++) {
		text += assignment.featurepairs[j].featlabel + " = " + 
			FeatureValueUtils.toString(assignment.featurepairs[j].featvalue) ;
		if (j < (assignment.featurepairs.length - 1)) {
			text += ", ";
		}
	}
	text += " ) = " + assignment.prob ;
	return text;
}
}
