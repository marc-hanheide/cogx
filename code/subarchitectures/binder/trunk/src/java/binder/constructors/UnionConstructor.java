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

package binder.constructors;

import java.util.Collection;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Vector;

import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.core.logging.ComponentLogger;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.PhantomProxy;
import binder.autogen.specialentities.RelationProxy;
import binder.autogen.specialentities.RelationUnion;
import binder.bayesiannetwork.BayesianNetworkManager;
import binder.utils.BinderUtils;
import binder.utils.FeatureValueUtils;
import binder.utils.ProbabilityUtils;


/**
 * Library to construct new binding unions, and compute their related probability
 * distributions (based on the information contained in the included proxies, and
 * the bayesian network to compute the internal coherence of the whole)
 * 
 * @author Pierre Lison
 * @version 24/09/2009 (started 15/08/2009)
 */

public class UnionConstructor  {

	
	// flag to activate error logging
	public static boolean ERRLOGGING = true;

	// flag to activate logging
	public static boolean LOGGING = false;
	
	private static Logger logger = ComponentLogger.getLogger(UnionConstructor.class);

	
	public boolean MERGE_FEATURES = false;
	
	
	// the Bayesian network manager
	BayesianNetworkManager BNManager;

	// Constant controlling the greediness of the binder
	// (higher == greedier binder)
	private float ALPHA_CONST = 0.2f;
	
	

	
	// ================================================================= 
	// INITIALISATION & CONFIGURATION METHODS   
	// ================================================================= 
	
	
	/**
	 *  Construct a new union constructor, bayed on the specification of 
	 *  a bayesian network
	 *  
	 * @param bayesiannetworkfile the file containing the bayesian network
	 */
	
	public UnionConstructor(String bayesiannetworkfile) {
		BNManager = new BayesianNetworkManager(bayesiannetworkfile);
	}

	
	/**
	 * Set the alpha parameter to control the binder greediness (higher --> greedier binder)
	 * 
	 * @param alpha the alpha parameter
	 */
	
	public void setAlphaParam(float alpha) {
		ALPHA_CONST = alpha;
	}
	

	// ================================================================= 
	// UNION CONSTRUCTION METHODS   
	// ================================================================= 
	
	

	/**
	 * Construct an initial, single-proxy union with a new data ID, and a timestamp
	 * 
	 * @param proxy the proxy to include in the initial union
	 * @param newDataID the entity ID of this new union
	 * @param timestamp the timestamp
	 * @return the new union
	 */
	
	public Union constructInitialUnion(Proxy proxy, String newDataID, CASTTime timestamp) {
		Vector<PerceivedEntity> curProxyV = new Vector<PerceivedEntity>();
		curProxyV.add(proxy);
		Union union = constructNewUnion(curProxyV, newDataID, timestamp);
		return union;
	} 
	

	/**
	 * Construct a new union based on the merge of several perceptual 
	 * entities (which may be proxies or unions themselves)
	 * 
	 * @param includedEntities as vector of entities (with size > 0 )
	 * @return the new union
	 */	
	public Union constructNewUnion
	(Vector<PerceivedEntity> includedEntities, String entityID, CASTTime timestamp) {

		if (BinderUtils.induceRelationUnion(includedEntities)) {
			return constructNewRelationUnion(includedEntities, entityID, timestamp);
		}
		else {
			return constructNewBasicUnion(includedEntities, entityID, timestamp);
		}

	}


	/**
	 * Construct a new basic union (i.e. not a relation union) based on 
	 * the merge of several perceptual entities (which may be proxies 
	 * or unions themselves)
	 * 
	 * @param includedEntities as vector of entities (with size > 0 )
	 * @return the new union
	 */

	private Union constructNewBasicUnion 
	(Vector<PerceivedEntity> includedEntities, String entityID, CASTTime timestamp) {
		//	log("***** Constructing a new union ****");


		// Specify the proxies included in the union
		Vector<Proxy> includedProxiesV = BinderUtils.getProxies(includedEntities);
		Proxy[] includedProxies = new Proxy[includedProxiesV.size()];
		includedProxies = includedProxiesV.toArray(includedProxies);
		
		// Extract the possible features for the union
		Collection<Feature> featuresC = getFeatures(includedEntities);
		Feature[] features = new Feature[featuresC.size()];
		features = featuresC.toArray(features);
		
		// Create a new union with a new data ID

		ProbabilityDistribution distrib = new ProbabilityDistribution();		
		Union union = new Union(entityID, 0.0f, timestamp, features, distrib, includedProxies);

		
		// Finally, compute the union distribution
		if  (includedEntities.size() == 1  && 
				features.length == includedEntities.elementAt(0).features.length) {
			union.distribution = includedEntities.elementAt(0).distribution;
		}
		else {
			union.distribution = computeUnionDistribution(union);
		} 

		return union;
	}


	/**
	 * Construct a new relation union based on the merge of several perceptual 
	 * entities (which may be proxies or unions themselves)
	 * 
	 * @param includedEntities as vector of entities (with size > 0 )
	 * @return the new union
	 */

	public RelationUnion constructNewRelationUnion 
	(Vector<PerceivedEntity> includedEntities, String entityID, CASTTime timestamp) {
		
		// Get the basic union
		Union bunion = constructNewBasicUnion(includedEntities, entityID, timestamp);

		// Copy the info of the basic union into a new relation union
		RelationUnion runion = BinderUtils.convertIntoRelationUnion(bunion);
		
		if (includedEntities.elementAt(0) instanceof RelationProxy) {
		runion.psource = ((RelationProxy)includedEntities.elementAt(0)).source;
		runion.ptarget = ((RelationProxy)includedEntities.elementAt(0)).target;
		}
		else if (includedEntities.elementAt(0) instanceof RelationUnion){
			runion.psource = ((RelationUnion)includedEntities.elementAt(0)).psource;
			runion.ptarget = ((RelationUnion)includedEntities.elementAt(0)).ptarget;	
		}

		return runion;

	}

	
	private Feature cloneFeature (Feature feat) {
		
		Feature newFeat = new Feature();
		newFeat.featlabel = feat.featlabel;
		newFeat.alternativeValues = new FeatureValue[feat.alternativeValues.length];
		
		for (int i = 0 ; i < feat.alternativeValues.length ; i++) {
			newFeat.alternativeValues[i] = FeatureValueUtils.cloneFeatureValue(feat.alternativeValues[i]);
		}
		return newFeat;
	}



	// ================================================================= 
	// UNION DISTRIBUTION METHODS   
	// ================================================================= 

	
	
	/** 
	 * Computes the probability distribution of the union, in three steps: 
	 * 		(1) computes the prior distribution of the union using the bayesian network, 
	 * 		(2) compute the proxy distributions for each proxy, and multiply them,  
	 * 		(3) finally, multiply the prior and the proxy distributions, and combines the result with a 
	 * 			multiplication factor to get the final distribution
	 * 
	 * @param union the union
	 * @return the probability distribution for the union
	 */
	
	public ProbabilityDistribution computeUnionDistribution(Union union) {
		
		/** STEP 1: Computes the prior distribution for the union */
		DiscreteProbabilityDistribution priorDistrib =  BNManager.getPriorDistribution(union);

		
		/** STEP 2: Compute the proxy distributions */
		Vector<CombinedProbabilityDistribution> proxiesDistrib = 
			new Vector<CombinedProbabilityDistribution>();

		// loop on the included proxies
		for (int i = 0 ; i < union.includedProxies.length ; i++) {
			Proxy proxy = union.includedProxies[i];

			// create a new combined probability distribution to specify the full proxy distribution
			OperationType opType = OperationType.MULTIPLIED;
			ProbabilityDistribution[] distributions = new DiscreteProbabilityDistribution[2];

			// define the first subdistribution to be the posterior distribution of the proxy
			distributions[0] = proxy.distribution;
	
			// computes the prior distribution of the proxy
			DiscreteProbabilityDistribution priorDistribForProxy = BNManager.getPriorDistribution(proxy);

			// inverts the result, and define it as the second subdistribution
			distributions[1] = 	ProbabilityUtils.invertDistribution(priorDistribForProxy);

			// adds the proxy distribution to the list of all proxy distributions
			CombinedProbabilityDistribution proxyDistrib = new CombinedProbabilityDistribution(distributions, opType);

			proxiesDistrib.add(proxyDistrib);
		}

		// create a new combined distribution, of size |proxiesDistrib| + 1
		OperationType fOpType = OperationType.MULTIPLIED;
		ProbabilityDistribution[] fDistributions = new ProbabilityDistribution[proxiesDistrib.size() + 1];

		// define the first subdistribution to be the prior union distribution 
		// multiplied by a particular factor (controlling the binder greediness)
		float multiplicationFactor = 1.0f / ((float)Math.pow(ALPHA_CONST, (union.includedProxies.length - 1)));
		fDistributions[0] = 
			ProbabilityUtils.multiplyDistributionWithConstantValue(priorDistrib, multiplicationFactor);

		// and define all the other subdistributions to be the proxy distributions
		for (int i = 1 ; i < proxiesDistrib.size() + 1 ; i++ ) {
			fDistributions[i] = proxiesDistrib.elementAt(i-1);
		}
		
		CombinedProbabilityDistribution finalDistrib = new CombinedProbabilityDistribution(fDistributions, fOpType);

		return finalDistrib;
	}



	 
	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 

	
	/**
	 * Get the complete set of features included in the entities
	 * 
	 * @param includedEntities the perceived entities
	 * @return the list of features
	 */
	
	private Collection<Feature> getFeatures (Vector<PerceivedEntity> includedEntities) {

		if (MERGE_FEATURES) {
		HashMap<String, Feature> features = new HashMap<String, Feature>();
		
		for (Enumeration<PerceivedEntity> e = includedEntities.elements(); e.hasMoreElements();) {
			PerceivedEntity prox = e.nextElement();
			for (int i = 0; i < prox.features.length ; i++) {
				Feature feat = new Feature();
				feat.featlabel = prox.features[i].featlabel;
				feat.alternativeValues = new FeatureValue[prox.features[i].alternativeValues.length];
				for (int j =0; j < prox.features[i].alternativeValues.length ; j++) {
					feat.alternativeValues[j] = 
						FeatureValueUtils.cloneFeatureValue(prox.features[i].alternativeValues[j]);
				}
				
				if (!features.containsKey(feat.featlabel)) {
					features.put(feat.featlabel, feat);
				}
				else {
					Feature mergedFeature = mergeFeatures (features.get(feat.featlabel), feat);
					features.remove(feat.featlabel);
					features.put(feat.featlabel, mergedFeature);
				}
			}
			

		}
		return features.values();
		}
		else {
			Vector<Feature> features = new Vector<Feature>();
			
			for (Enumeration<PerceivedEntity> e = includedEntities.elements(); e.hasMoreElements();) {
				PerceivedEntity prox = e.nextElement();
				for (int i = 0; i < prox.features.length ; i++) {
					Feature feat = new Feature();
					feat.featlabel = prox.features[i].featlabel;
					feat.alternativeValues = new FeatureValue[prox.features[i].alternativeValues.length];
					for (int j =0; j < prox.features[i].alternativeValues.length ; j++) {
						feat.alternativeValues[j] = 
							FeatureValueUtils.cloneFeatureValue(prox.features[i].alternativeValues[j]);
					}
					
					features.add(feat);
				}
				

			}
			return features;
		}
	}
	
	
	private static Feature mergeFeatures (Feature feat1, Feature feat2) {
		
		Feature mergedFeature = new Feature();
		mergedFeature.featlabel = feat1.featlabel;
		if (feat1.alternativeValues.length == 1 && feat2.alternativeValues.length == 1) {
			mergedFeature.alternativeValues = new FeatureValue[1];
			mergedFeature.alternativeValues[0] = 
				FeatureValueUtils.mergeFeatureValues(feat1.alternativeValues[0], feat2.alternativeValues[0]);
		}
		else {
			errlog("WARNING: feature merging of this type are not implemented yet!");
		}
		return mergedFeature;
	}
	
	
	public BayesianNetwork getBayesianNetwork () {
		return BNManager.getBayesianNetwork();
	}
	
	/**
	public static Feature createSpecialBindingFeature (PhantomProxy phantom) {
		Feature feat = new Feature();
		feat.featlabel = "boundPhantom";
		feat.alternativeValues = new AddressValue[1];
		feat.alternativeValues[0] = new AddressValue(1.0f, phantom.timeStamp, phantom.entityID);
		return feat;
	}*/
	
	
	private static void log (String s) {
		if (LOGGING)
		logger.debug("[UnionConstructor] " + s);
	}
	
	private static void errlog (String s) {
		if (ERRLOGGING)
			logger.debug("[UnionConstructor] " + s);
	}
}
