// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

package binder.utils;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Iterator;
import java.util.Vector;

import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.core.logging.ComponentLogger;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.FloatValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.featvalues.UnknownValue;
import binder.autogen.specialentities.PhantomProxy;
import binder.autogen.specialentities.RelationProxy;
import binder.autogen.specialentities.RelationUnion;
import binder.constructors.DistributionGeneration;
import binder.constructors.ProxyConstructor;
import binder.filtering.EntityFilter;


/**
 * Utility library for the core binder mechanism
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 01/09/2009)
 * 
 */

public class BinderUtils {
	
	
	// minimum threshold above which unknown values can be created in features
	public static float MINIMUM_PROB_OF_UNKNOWN_FEATVALUES = 0.25f;

	public static boolean ADD_DEFAULT_SALIENCY = false;
	public static String DEFAULT_SALIENCY = "high";
	
	private static Logger logger = ComponentLogger.getLogger(BinderUtils.class);

	
	// ================================================================= 
	// EXISTENCE PROBABILITY METHODS   
	// ================================================================= 

	

	/**
	 * Set the existence probabilities to each union contained in the union configurations
	 * 
	 * @param configs the configurations
	 */
	public static void addProbExistsToUnions (Vector<UnionConfiguration> configs) {

		// extract the unions
		ArrayList<Union> unions = getUnions (configs);

		// set the existence probabilities in the unions
		setProbExistUnions(unions, configs);
	}



	/**
	 * Set the existence probabilities in the unions, given the (normalised) configuration 
	 * probabilities in the union configurations
	 * 
	 * @param unions the unions
	 * @param configs the union configurations (with normalised probabilities)
	 */
	
	public static void setProbExistUnions 
	(ArrayList<Union> unions, Vector<UnionConfiguration> configs) {

		for (Iterator<Union> e = unions.iterator() ; e.hasNext() ; ) {
			Union u = e.next();

			u.probExists = 0.0f;
			for (Enumeration<UnionConfiguration> f = configs.elements() ; f.hasMoreElements() ;) {
				UnionConfiguration config = f.nextElement();
				if (isUnionInConfig(config, u)) {
					u.probExists += config.configProb;
				}
			}
		}
		
		
		for (Iterator<Union> e = unions.iterator() ; e.hasNext() ; ) {
			Union u = e.next();
			if (u.probExists == 0.0f) {
				u.probExists = 0.000000000001f;
			}
		}
	}


	
	// ================================================================= 
	// PROXY COMPLETION METHODS   
	// ================================================================= 
	
	 
	/**
	 * Complete the proxy with additional information: 
	 * 	1) if addUnknowns == true, add the unknown feature values to the features 
	 *  2) generate the probability distribution for the proxy (assuming the independent
	 *     probabilities for each feature value have been set)
	 *     
	 * @param proxy the proxy
	 * @param addUnknowns whether to add unknown feature values
	 */
	
	public static void completeProxy (Proxy proxy, boolean addUnknowns, int proxyDistribFilter) {
		// If necessary, add unknown values
		if (addUnknowns) {
			addUnknownFeatureValues(proxy.features);
		}

		
		if (ADD_DEFAULT_SALIENCY && !hasFeature(proxy, "saliency") && !(proxy instanceof PhantomProxy)) {
			addSaliencyFeature(proxy);
		}
		
		// if the probability distribution of the updated proxy is unavailable, regenerate it
	//	if (proxy.distribution == null ||  !(proxy.distribution instanceof DiscreteProbabilityDistribution)) {
		proxy.distribution = 
				DistributionGeneration.generateProbabilityDistribution(proxy);
			
			if (proxyDistribFilter > 0) {
				proxy.distribution = 
					EntityFilter.filterDistribution((DiscreteProbabilityDistribution)proxy.distribution, proxyDistribFilter);
			}
	//	}
	}
	
	
	public static void addSaliencyFeature (PerceivedEntity entity) {
		
		Feature[] newFeatures = new Feature[entity.features.length + 1];
		for (int i = 0 ; i < entity.features.length ; i++) {
			newFeatures[i] = entity.features[i];
		}
		StringValue stringValue = ProxyConstructor.createStringValue(DEFAULT_SALIENCY, 1.0f);
		ProxyConstructor.setTimeStamp(stringValue, entity.timeStamp);
		newFeatures[entity.features.length] = 
			ProxyConstructor.createFeatureWithUniqueFeatureValue("saliency", stringValue);
		
		entity.features = newFeatures;
	}
	
	
	public static boolean hasFeature(PerceivedEntity entity, String featlabel) {
		for (int i = 0 ; i < entity.features.length ; i++) {
			if (entity.features[i].featlabel.equals(featlabel)) {
				return true;
			}
		}
		return false;
	}

	
	/**
	 * If necessary, add unknown feature values to the feature.  The parameter 
	 * MINIMUM_PROB_OF_UNKNOWN_FEATVALUES determines the threshold above which an 
	 * unknown feature value will be added to the feature values
	 * 
	 * @param features the list of features
	 */

	public static void addUnknownFeatureValues (Feature[] features) {

		// loop on the features
		for (int i = 0 ; i < features.length ; i++) {

			boolean alreadyIncludesUnknownValue = false;

			// sum up the probabilities of each feature value for the feature
			float totalProb = 0.0f;
			Vector<FeatureValue> values = new Vector<FeatureValue>();
			for (int j= 0 ; j < features[i].alternativeValues.length ; j++) {
				values.add(features[i].alternativeValues[j]);
				totalProb += features[i].alternativeValues[j].independentProb;
				if (FeatureValueUtils.isUnknownValue(features[i].alternativeValues[j])) {
					alreadyIncludesUnknownValue = true;
				}
			}

			// If the unknown feature value is likely enough, add it to the feature values set
			if (!alreadyIncludesUnknownValue && totalProb < (1.0f - MINIMUM_PROB_OF_UNKNOWN_FEATVALUES) ) {
				features[i].alternativeValues = new FeatureValue[values.size() + 1];
				for (int j = 0 ; j < values.size(); j++ ) {
					features[i].alternativeValues[j] = values.elementAt(j);
				}
				features[i].alternativeValues[values.size()] = 
					new UnknownValue((1.0f - totalProb), features[i].alternativeValues[0].timeStamp);
			}
		}
	}

	

	// ================================================================= 
	// RELATION UNION METHODS   
	// ================================================================= 


	/**
	 * Convert a normal union into a relation union (without source and target)
	 * 	
	 * @param bunion the normal union
	 * @return the relation union
	 */

	public static RelationUnion convertIntoRelationUnion (Union bunion) {

		String entityID = bunion.entityID;
		Feature[] features = bunion.features;
		ProbabilityDistribution distribution = bunion.distribution;
		Proxy[] includedProxies = bunion.includedProxies;
		float probExists = bunion.probExists;
		CASTTime timeStamp = bunion.timeStamp;

		Feature psource = new Feature("psource", new FeatureValue[0]);
		Feature ptarget = new Feature("ptarget", new FeatureValue[0]);
		Feature usource = new Feature("usource", new FeatureValue[0]);
		Feature utarget = new Feature("utarget", new FeatureValue[0]);
		
		RelationUnion runion = new RelationUnion(entityID, probExists, timeStamp, 
				features, distribution, includedProxies, psource, ptarget, usource, utarget);
		  
		return runion;
	}



	/**
	 * Returns true if all the entities are relation proxies or contain relation proxies
	 * (and hence induce a relation union), false otherwise
	 * 
	 * @param entities the entities
	 * @return true if a relation union is induced, false otherwise
	 */

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



	// ================================================================= 
	// PRETTY PRINT UTILITY METHODS   
	// ================================================================= 
	
	
	/**
	 * Return a string-formatted version of the discrete probability distribution
	 * 
	 * @param distrib the distribution
	 * @return the string representing the distrib
	 */
	
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

	
	/**
	 * Return a string-formatted version of the discrete probability assignment
	 * 
	 * @param assignment the assignment
	 * @return the string representing the assignment
	 */
	
	public static String getPrettyPrintProbabilityAssignment
	(DiscreteProbabilityAssignment assignment) {

		if (assignment == null) {
			return "";
		}
		else if (assignment.featurepairs == null) {
			return "";
		}
		
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
	
	
	
	// ================================================================= 
	// UNIONS AND PROXY EXTRACTION METHODS   
	// ================================================================= 
	
	
	/**
	 * Get all the proxies included in a set of perceptual entities (in case the entity
	 * is a proxy, it is simply added, and in case it is an union, the set of all included
	 * proxies is added to the resulting set)
	 * 
	 * @param includedEntities the set of perceptual entities
	 * @return the resulting set of proxies
	 */
	public static Vector<Proxy> getProxies (Vector<PerceivedEntity> includedEntities) {
		Vector<Proxy> includedProxies = new Vector<Proxy>();
		
		// loop on the entities
		for (Enumeration<PerceivedEntity> en = includedEntities.elements() ; en.hasMoreElements() ;) {
			PerceivedEntity entity = en.nextElement();
			
			// if the entity is a proxy, simply add it
			if (entity instanceof Proxy) {
				includedProxies.add((Proxy)entity);
			}
			
			// if it is an union, add all included proxies
			else if (entity instanceof Union) {
				Union includedUnion = (Union)entity;
				for (int i = 0 ; i < includedUnion.includedProxies.length ; i++) {
					includedProxies.add(includedUnion.includedProxies[i]);
				}
			}
		}
		return includedProxies;
	}
	
	

	/**
	 * Get the list of all (distinct) unions contained in the union configurations
	 * 
	 * @param configs the union configurations
	 * @return the list of unions
	 */
	public static ArrayList<Union> getUnions(Vector<UnionConfiguration> configs) {

		ArrayList<Union> unions = new ArrayList<Union>();

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {

			UnionConfiguration config = e.nextElement();
			for (int i = 0; i < config.includedUnions.length ; i++) {

				Union union = config.includedUnions[i];

				if (!unions.contains(union)) {
					unions.add(union);
				}
			}
		}

		return unions;
	}

	
	// ================================================================= 
	// GENERIC UTILITY METHODS   
	// ================================================================= 
	
	
	/**
	 * Returns true is the union is in the union configuration, false otherwise
	 * 
	 * @param config the union configuration
	 * @param union the union
	 * @return true if in config, false otherwise
	 */
	
	private static boolean isUnionInConfig(UnionConfiguration config, Union union) {
		for (int i = 0 ; i < config.includedUnions.length ; i++) {
			if (config.includedUnions[i].equals(union)) {
				return true;
			}
		}
		return false;
	} 
	
	

	/**
	 * Check if the union is already contained (maybe incorporated in a different
	 * java object) in the union list
	 * 
	 * @param unions the list of unions
	 * @param union the union
	 * @return true if the union is in the list, false otherwise
	 */
	private static boolean isInList (ArrayList<Union> unions, Union union) {
		for (Iterator<Union> i = unions.iterator(); i.hasNext() ;) {
			Union u = i.next();
			if (u.entityID.equals(union.entityID)  && 
					u.includedProxies.length == union.includedProxies.length 
					&& u.timeStamp == union.timeStamp) {
				return true;
			}
		}
		return false;
	}

}
