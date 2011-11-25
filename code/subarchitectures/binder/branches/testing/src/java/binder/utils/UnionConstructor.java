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

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.specialentities.RelationProxy;
import binder.autogen.specialentities.RelationUnion;
import binder.bayesiannetwork.BayesianNetworkManager;

public class UnionConstructor  {


	// The Bayesian network manager
	BayesianNetworkManager BNManager;

	// Constant controlling the greediness of the binder
	// (lower == greedier binder)
	public float ALPHA_CONST = 0.2f;

	// Prior probability of the existence of a proxy
	public float PRIOR_PEXISTS = 0.4f;
	
	public static boolean LOGGING = true;
	
	
	public UnionConstructor(String bayesiannetworkfile) {
		BNManager = new BayesianNetworkManager(bayesiannetworkfile);
	}

	
	public void setAlphaParam(float alpha) {
		ALPHA_CONST = alpha;
	}
	

	public void setPriorParam(float prior) {
		PRIOR_PEXISTS = prior;
	}


	/**
	 * Construct a new union based on the merge of several perceptual 
	 * entities (which may be proxies or unions themselves)
	 * 
	 * @param includedEntities as vector of entities (with size > 0 )
	 * @return the new union
	 */	
	public Union constructNewUnion
	(Vector<PerceivedEntity> includedEntities, String entityID) {

		if (BinderUtils.induceRelationUnion(includedEntities)) {
			return constructNewRelationUnion(includedEntities, entityID);
		}
		else {
			return constructNewBasicUnion(includedEntities, entityID);
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
	(Vector<PerceivedEntity> includedEntities, String entityID) {
		//	log("***** Constructing a new union ****");

		// Create a new union with a new data ID
		Union union = new Union() ;

		union.entityID = entityID;
		union.timeStamp = System.currentTimeMillis();

		// Specify the proxies included in the union
		Vector<Proxy> includedProxies = BinderUtils.getProxies(includedEntities);
		union.includedProxies = new Proxy[includedProxies.size()];
		union.includedProxies = includedProxies.toArray(union.includedProxies);

		// Compute the existence probability of the union
	//	union.probExists = computeProbExists(includedEntities);

		// Extract the possible features for the union
		Vector<Feature> features = getFeatures(includedEntities);
		union.features = new Feature[features.size()];
		union.features = features.toArray(union.features);

		// Finally, compute the union distribution
		if (includedEntities.size() > 1) {
			union.distribution = computeUnionDistribution(union);
		}
		else {
			union.distribution = includedEntities.elementAt(0).distribution;
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
	(Vector<PerceivedEntity> includedEntities, String entityID) {
		
		// Get the basic union
		Union bunion = constructNewBasicUnion(includedEntities, entityID);

		// Copy the info of the basic union into a new relation union
		RelationUnion runion = BinderUtils.convertIntoRelationUnion(bunion);
		
		// INCORRECT - should change this at some point to handle merged relation unions
		runion.source = ((RelationProxy)includedEntities.elementAt(0)).source ;
		runion.target = ((RelationProxy)includedEntities.elementAt(0)).target ;

		return runion;

	}

	
	
	private Vector<Feature> getFeatures (Vector<PerceivedEntity> includedEntities) {
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



	private float computeProbExists (Vector<PerceivedEntity> includedEntities) {
		float probExists = PRIOR_PEXISTS;
		float probNotExists = (1 - PRIOR_PEXISTS);

		for (Enumeration<PerceivedEntity> e = includedEntities.elements(); e.hasMoreElements();) {
			PerceivedEntity prox = e.nextElement();
			probExists = probExists * prox.probExists; 
			probNotExists = probNotExists * (1- prox.probExists); 
		}	

		probExists = probExists / ((float) Math.pow(PRIOR_PEXISTS,(includedEntities.size())));
		probNotExists = probNotExists / ((float) Math.pow((1-PRIOR_PEXISTS),(includedEntities.size())));

		float normConstant = 1.0f / (probExists + probNotExists);
		probExists = normConstant * probExists;
		return probExists;
	}



	/** 
	 * Compute the probability distribution of the union
	 * @param union
	 * @return
	 */
	public ProbabilityDistribution computeUnionDistribution(Union union) {
		// Compute the prior distribution for the union
		DiscreteProbabilityDistribution priorDistrib =  BNManager.getPriorDistribution(union);

		//		log("Maximum for prior distribution of the union: " + GradientDescent.getMaximum(priorDistrib));

		Vector<CombinedProbabilityDistribution> proxiesDistrib = 
			new Vector<CombinedProbabilityDistribution>();

		for (int i = 0 ; i < union.includedProxies.length ; i++) {
			Proxy proxy = union.includedProxies[i];

			//		log("Maximum for observation-driven distribution of the proxy " + i +  ": "
			// + GradientDescent.getMaximum(proxy.distribution));

			DiscreteProbabilityDistribution priorDistribForProxy = BNManager.getPriorDistribution(proxy);

			//		log("Maximum for prior distribution of the proxy " + i +  
			// ": " + GradientDescent.getMaximum(priorDistribForProxy));

			CombinedProbabilityDistribution finalProxyDistrib = new CombinedProbabilityDistribution();
			finalProxyDistrib.opType = OperationType.MULTIPLIED;
			finalProxyDistrib.distributions = new DiscreteProbabilityDistribution[2];
			finalProxyDistrib.distributions[0] = proxy.distribution;
			
			finalProxyDistrib.distributions[1] = 
				ProbabilityUtils.invertDistribution(priorDistribForProxy);

			finalProxyDistrib.distributions[1] = 
				ProbabilityUtils.multiplyDistributionWithConstantValue
				((DiscreteProbabilityDistribution)finalProxyDistrib.distributions[1], ALPHA_CONST);

			//		log("Maximum for final distribution of the proxy " + 
			// i +  ": " + GradientDescent.getMaximum(finalProxyDistrib));

			proxiesDistrib.add(finalProxyDistrib);
		}

		CombinedProbabilityDistribution finalDistrib = new CombinedProbabilityDistribution();
		finalDistrib.opType = OperationType.MULTIPLIED;
		finalDistrib.distributions = new ProbabilityDistribution[proxiesDistrib.size() + 1];

		finalDistrib.distributions[0] = 
			ProbabilityUtils.multiplyDistributionWithConstantValue(priorDistrib, ALPHA_CONST);

		for (int i = 1 ; i < proxiesDistrib.size() + 1 ; i++ ) {
			finalDistrib.distributions[i] = proxiesDistrib.elementAt(i-1);
		}

		//		log("Maximum for final distribution of the union: " + GradientDescent.getMaximum(finalDistrib));

		return finalDistrib;
	}




	public Union getInitialUnion(Proxy proxy, String newDataID) {
		Vector<PerceivedEntity> curProxyV = new Vector<PerceivedEntity>();
		curProxyV.add(proxy);
		Union union = constructNewUnion(curProxyV, newDataID);
		return union;
	} 

	private void log (String s) {
		System.out.println("[UnionConstructor] " + s);
	}
}