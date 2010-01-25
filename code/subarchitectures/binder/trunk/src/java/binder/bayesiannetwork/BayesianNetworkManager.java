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


package binder.bayesiannetwork;

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Vector;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.core.Feature;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.utils.BinderUtils;
import binder.utils.FeatureValueUtils;
import binder.filtering.EntityFilter;
import binder.filtering.MaximumSearch;
import binder.utils.ProbabilityUtils;


/**
 * Class for managing a bayesian network expressing feature correlations 
 * between features
 * 
 * TODO: Reintegrate the cache functionality
 * 
 * @author Pierre Lison
 * @version 09/09/2009 (started 01/08/2009)
 */

public class BayesianNetworkManager {

	// The bayesian network wrapper
	private BayesianNetworkWrapper network;

	// Turn logging on/off
	boolean LOGGING = false;

	private static Logger logger = ComponentLogger.getLogger(BayesianNetworkManager.class);

	
	// Cache of already computed distributions for a given entity
	private HashMap<PerceivedEntity,DiscreteProbabilityDistribution> alreadyComputedDistribs;



	// =================================================================
	// INITIALISATION
	// =================================================================


	/**
	 * Initialise the bayesian network manager
	 */

	public BayesianNetworkManager(String configurationFile) {		

		log("Start building the bayesian network...");
		log("Configuration file used: " + configurationFile);

		network = new BayesianNetworkWrapper (configurationFile);

		log("Construction of bayesian network successfull!");

		if (network.getNodes() == null) {
			log("WARNING: number of nodes is null!");
		}

		log("number of nodes: " + network.getNodes().length);		
		log("number of edges: " + network.getEdges().length);

		// Initialize the list of already computed prior distributions
		alreadyComputedDistribs = new HashMap<PerceivedEntity, DiscreteProbabilityDistribution>();

	}


	// =================================================================
	// METHODS FOR COMPUTING PRIOR DISTRIBUTIONS
	// =================================================================


	/**
	 * Compute the prior distribution for an entity, based on a bayesian network
	 * 
	 * NOTE: only limited to discrete probability distributions for now
	 * 
	 * @param entity the entity
	 * @return the probability distribution
	 */
	public DiscreteProbabilityDistribution getPriorDistribution(PerceivedEntity entity) {

		// Check if the prior distribution has already been computed for the proxy
		if (false && alreadyComputedDistribs.containsKey(entity)) {
			return alreadyComputedDistribs.get(entity);
		}
		else {

			DiscreteProbabilityDistribution distrib  ;

			// distinct computation mechanisms for proxies and unions
			if (entity instanceof Union) {		
				distrib = getPriorDistributionForUnion ((Union) entity);
			}

			else {
				distrib = getPriorDistributionForProxy ((Proxy)entity);			
			}

			// And add the result to the list of already compute distributions
			alreadyComputedDistribs.put(entity, distrib);

			return distrib;
		}
	}


	
	// =================================================================
	// PRIOR DISTRIBUTIONS FOR PROXIES
	// =================================================================

	
	/**
	 * Compute the prior distribution for a given set of features in a proxy, based on the 
	 * bayesian network
	 * 
	 * @param features the set of features
	 * @return the probability distribution
	 */
	private DiscreteProbabilityDistribution getPriorDistributionForProxy (Proxy proxy) {

		log("Start computing the prior distribution...");
		DiscreteProbabilityDistribution priorDistrib = new DiscreteProbabilityDistribution();

		// Set of probability assignments per features
		Vector<DiscreteProbabilityAssignment[]> priorDistribsForFeature = 
			new Vector<DiscreteProbabilityAssignment[]>();

			if (proxy.features.length > 0) {
				// Loop on the features array
				for (int i = 0 ; i < proxy.features.length; i ++) {
					log("Current feature: \"" + proxy.features[i].featlabel + "\"");

					// Get the set of probability assignment for the feature
					DiscreteProbabilityAssignment[] priorDistribForFeature = 
						getPriorDistribution(proxy.features[i]);
					priorDistribsForFeature.add(priorDistribForFeature);
				}

				// Compute the joint probability distribution
				Vector<DiscreteProbabilityAssignment> jointDistribV = 
					computeJointPriorDistribution(priorDistribsForFeature);

				// And specify the assignments defining the distribution
				priorDistrib.assignments = new DiscreteProbabilityAssignment[jointDistribV.size()];
				priorDistrib.assignments = jointDistribV.toArray(priorDistrib.assignments);
			} 
			else {
				log("WARNING, feature length is == 0 !");
				priorDistrib.assignments = new DiscreteProbabilityAssignment[1];
				priorDistrib.assignments[0] = new DiscreteProbabilityAssignment();
				priorDistrib.assignments[0].featurepairs = new FeatureValuePair[0];
				priorDistrib.assignments[0].prob = 1.0f;
			}

			return priorDistrib;
	}


	
	// =================================================================
	// PRIOR DISTRIBUTIONS FOR UNIONS
	// =================================================================


	private DiscreteProbabilityDistribution getPriorDistributionForUnion  (Union union) {
		
		DiscreteProbabilityDistribution[] priorDistribs = new DiscreteProbabilityDistribution[union.includedProxies.length];
		
		for (int i = 0 ; i < priorDistribs.length ; i++) {
			priorDistribs[i] = getPriorDistribution (union.includedProxies[i]);
		}

		return computeJointPriorDistribution(priorDistribs);
	}	



	// =================================================================
	// METHODS FOR COMPUTING JOINT DISTRIBUTIONS
	// =================================================================




	private DiscreteProbabilityDistribution computeJointPriorDistribution 
	(DiscreteProbabilityDistribution[] priorDistribs) {

		DiscreteProbabilityDistribution jointDistrib = null ;
		
		// easy case: union of two proxies
		if (priorDistribs.length == 2) {
			jointDistrib = computeJointPriorDistribution (priorDistribs[0], priorDistribs[1]);	
		}

		// more than 2 proxies (recursive call)
		else if (priorDistribs.length > 2) {

			DiscreteProbabilityDistribution[] firstTwo = new DiscreteProbabilityDistribution[2];
			firstTwo[0] = priorDistribs[0];
			firstTwo[1] = priorDistribs[1];

			DiscreteProbabilityDistribution distribForFirstTwo = computeJointPriorDistribution (firstTwo);
			
			DiscreteProbabilityDistribution[] newPriorDistribs = 
				new DiscreteProbabilityDistribution[priorDistribs.length - 1];
			
			newPriorDistribs[0] = distribForFirstTwo;

			for (int z = 1 ; z < newPriorDistribs.length ; z++) {
				newPriorDistribs[z] = priorDistribs[z+1];
			}

			jointDistrib = computeJointPriorDistribution (newPriorDistribs);
		}

		// and the easiest case  :-)
		else if (priorDistribs.length == 1){
			jointDistrib = priorDistribs[0];
		}

		return jointDistrib;
	}




	private DiscreteProbabilityDistribution computeJointPriorDistribution 
		(DiscreteProbabilityDistribution distrib1, DiscreteProbabilityDistribution distrib2) {
		
		Vector<DiscreteProbabilityAssignment> jointAsses = new Vector<DiscreteProbabilityAssignment>();

		for (int i = 0 ; i< distrib1.assignments.length ; i++) {

			DiscreteProbabilityAssignment ass1 = distrib1.assignments[i];			
			Vector<FeatureValuePair> pairsInAss1 = getFeatureValuePairs (ass1);

			for (int j = 0 ; j < distrib2.assignments.length;  j++) {

				DiscreteProbabilityAssignment ass2 = distrib2.assignments[j];

				Vector<FeatureValuePair> pairs = getFeatureValuePairs (ass2);
				pairs.addAll(pairsInAss1);

				double jointProbValue = getJointProbabilityValue(pairs);			
				DiscreteProbabilityAssignment jointAss = createNewAssignment(pairs, jointProbValue);
				jointAsses.add(jointAss);
			}
		}

		DiscreteProbabilityDistribution jointDistrib = new DiscreteProbabilityDistribution();
		jointDistrib.assignments = new DiscreteProbabilityAssignment[jointAsses.size()];
		jointDistrib.assignments = jointAsses.toArray(jointDistrib.assignments);

		
		// filtering
		DiscreteProbabilityDistribution[] priorDistribs = new DiscreteProbabilityDistribution[2];
		priorDistribs[0] = distrib1;
		priorDistribs[1] = distrib2;
		jointDistrib = filterDistribution (jointDistrib, priorDistribs);

		
		return jointDistrib;
	}
	
	

	/**
	 * Compute the joint probability distribution (defined as a set of probability 
	 * assignments) given the prior distributions
	 * 
	 * @param priorDistribs the prior distributions
	 * @return the set of probability assignments
	 */
	private Vector<DiscreteProbabilityAssignment> computeJointPriorDistribution 
	(Vector<DiscreteProbabilityAssignment[]> priorDistribs) {

		log("Computing the joint distribution, vector size: " + priorDistribs.size());

		// Case 1: there is only one prior distribution
		if (priorDistribs.size() == 1) {

			Vector<DiscreteProbabilityAssignment> finalAssignment = 
				new Vector<DiscreteProbabilityAssignment>();

			DiscreteProbabilityAssignment[] curAssignments = priorDistribs.elementAt(0);

			for (int i = 0; i < curAssignments.length ; i++) {
				finalAssignment.add(priorDistribs.elementAt(0)[i]);
			}
			return finalAssignment;
		}

		// Case 2: there more than one prior distribution (recursive definition)
		else if (priorDistribs.size() > 1){

			// Remove one of the prior distribution
			DiscreteProbabilityAssignment[] independentProbs = priorDistribs.remove(0);

			// Recursively compute the joint distribution with all distributions except
			// the one we just removed
			Vector<DiscreteProbabilityAssignment> previousAssignments = 
				computeJointPriorDistribution(priorDistribs);

			// Initialize the new set of probability assignments
			Vector<DiscreteProbabilityAssignment> newAssignments = 
				new Vector<DiscreteProbabilityAssignment>();

			// Loop on the set of assignments specified in independentProbs
			for (int i = 0 ; i < independentProbs.length; i++) {

				DiscreteProbabilityAssignment assignment1 = independentProbs[i];

				// Loop on the set of assignments specified in previousAssignments
				for (Enumeration<DiscreteProbabilityAssignment> e = 
					previousAssignments.elements(); e.hasMoreElements() ; ) {

					DiscreteProbabilityAssignment assignment2 = e.nextElement();

					// Construct a sequence of feature pairs merging the feature pairs of 
					// assignment1 and assignment2
					Vector<FeatureValuePair> featpairs = new Vector<FeatureValuePair>();
					for (int j = 0 ; j < assignment1.featurepairs.length ; j++) {
						featpairs.add(assignment1.featurepairs[j]);
					}
					for (int j = 0 ; j < assignment2.featurepairs.length ; j++) {
						featpairs.add(assignment2.featurepairs[j]);
					}

					// Get the joint probability value for featpairs
					float prob = (float) getIndependentJointProbabilityValue (featpairs); 

					// create a probability assignment for featpairs and add it to the new assignments
					DiscreteProbabilityAssignment newAssignment = new DiscreteProbabilityAssignment();
					newAssignment.featurepairs = new FeatureValuePair[featpairs.size()];
					newAssignment.featurepairs = featpairs.toArray(newAssignment.featurepairs);
					newAssignment.prob = prob;
					newAssignments.add(newAssignment);
					log("prob: " + prob);
				}
			}

			return newAssignments;
		}

		return new Vector<DiscreteProbabilityAssignment>();
	}




	/**
	 * Compute the joint probability value for the given set of feature-value pairs
	 * @param featpairs the feature value pairs
	 * 
	 * @return the probability value
	 */
	public double getIndependentJointProbabilityValue (Vector<FeatureValuePair> featpairs) {

		double result = 1.0f;

		// Loop on the feature value pairs
		for(Enumeration<FeatureValuePair> e = featpairs.elements(); e.hasMoreElements(); ) {
			FeatureValuePair featpair1 = e.nextElement();

			// Compute the final probability for featpair1
			double finalProbForFeatpair1;

			// If featpair1 is conditionally independent from all the other features
			finalProbForFeatpair1 = getIndependentProb(featpair1.featlabel, featpair1.featvalue);

			// Integrate the probability of featpair1 into the total result
			result = result * finalProbForFeatpair1;
		}

		log("joint probability value: " + result);
		return result;
	}
	

	/**
	 * Compute the joint probability value for the given set of feature-value pairs
	 * @param featpairs the feature value pairs
	 * 
	 * @return the probability value
	 */
	public double getJointProbabilityValue(Vector<FeatureValuePair> featpairs) {

		double result = 1.0f;

		// Loop on the feature value pairs
		for(Enumeration<FeatureValuePair> e = featpairs.elements(); e.hasMoreElements(); ) {
			FeatureValuePair featpair1 = e.nextElement();

			// conditional probability of featpair1 given all the other features
			double condProbs = 1.0f;

			// counts of correlations between featpair1 and the other feature pairs
			int nbConds = 0;

			// Loop a second time on the feature value pairs
			for(Enumeration<FeatureValuePair> f = featpairs.elements(); f.hasMoreElements() ; ) {

				FeatureValuePair featpair2 = f.nextElement();
				if (featpair1 != featpair2) {

					// Get the correlation between the two feature pairs, given
					// the specifications of the bayesian network
					FeatureValueCorrelation corr = network.getCorrelationsBetweenFVs(featpair1, featpair2);
					if (corr != null) {

						// If a correlation is found, update the condition probability
						condProbs = condProbs * corr.condProb;
						nbConds++;
					}
				}
			}

			// Compute the final probability for featpair1
			double finalProbForFeatpair1;
			if (condProbs < 1.0f) {
				finalProbForFeatpair1 = 0.0f;
				if (nbConds == 1 ) {
					finalProbForFeatpair1 = condProbs;
				}
				if (nbConds == 2 ) {
					finalProbForFeatpair1 = Math.sqrt(condProbs);
				}
				if (nbConds == 3 ) {
					finalProbForFeatpair1 = Math.cbrt(condProbs);
				}
				else {
					log("WARNING: not implemented yet");
				}
			}
			// If featpair1 is conditionally independent from all the other features
			else {
				finalProbForFeatpair1 = getIndependentProb(featpair1.featlabel, featpair1.featvalue);
			}

			// Integrate the probability of featpair1 into the total result
			
			result = result * finalProbForFeatpair1;
		}

		log("joint probability value: " + result);
		return result;
	}


	/**
	 * Get the conditionally independent probability of a feature value pair
	 * based on the bayesian network
	 * @param featlabel the feature label
	 * @param featvalue the feature value
	 * @return the probability value (= 0.5f if feature pair not found in the network)
	 */
	public float getIndependentProb (String featlabel, FeatureValue featvalue) {

		// Search for the node in the bayesian network
		BayesianNetworkNode node = network.getNodeWithFeature(featlabel);

		if (node != null) {

			// Loop on the possible values in the bayesian network
			for (int i = 0; i < node.feat.alternativeValues.length ; i++) {
				FeatureValue featvalue2 = node.feat.alternativeValues[i];

				// If one feature value in the bayesian network matches the provided 
				// feature value and has a specified probability, just return its value

				if (FeatureValueUtils.haveEqualValue(featvalue, featvalue2)) {
					if (featvalue2.independentProb > 0) {
						return featvalue2.independentProb;
					}
					else {
						return (1.0f / node.feat.alternativeValues.length);   // INCORRECT
					}
				}
			}
		}

		log("WARNING: Independent probability for " + featlabel + " = " + FeatureValueUtils.toString(featvalue) + " is not specified");
		return 0.5f;
	}



	// =================================================================
	// UTILITIES
	// =================================================================



	private Vector<FeatureValuePair> getFeatureValuePairs(DiscreteProbabilityAssignment ass) {

		Vector<FeatureValuePair> pairs = new Vector<FeatureValuePair>();
		for (int y = 0 ; y < ass.featurepairs.length ; y++) {
			pairs.add(ass.featurepairs[y]);
		}

		return pairs;
	}

	private DiscreteProbabilityAssignment createNewAssignment (Vector<FeatureValuePair> pairs, double probValue) {

		DiscreteProbabilityAssignment ass = new DiscreteProbabilityAssignment();
		ass.featurepairs = new FeatureValuePair[pairs.size()];
		for (int i = 0 ; i < pairs.size() ; i++ ) {
			ass.featurepairs[i] = pairs.elementAt(i);
		}
		ass.prob = (float) probValue;

		return ass;
	}

	/**
	 * Get the array of probability assignments for a particular feature
	 * (one for each possible feature value, together with its probability)
	 * 
	 * @param feature the feature
	 * @return array of probability assignments
	 */
	public DiscreteProbabilityAssignment[] getPriorDistribution (Feature feature) {

		// Ensuring the set of features values is not null
		if (feature.alternativeValues != null) {

			// Create the array of assignments
			DiscreteProbabilityAssignment[] assignments = 
				new DiscreteProbabilityAssignment[feature.alternativeValues.length];

			log("Computing prior distribution with feature: \"" + feature.featlabel + "\"");

			// Looping on the set of alternative feature values
			for (int i = 0; i < feature.alternativeValues.length; i++) {

				FeatureValue featvalue = feature.alternativeValues[i];
				log("Feature value currently looked at: " + FeatureValueUtils.toString(featvalue));

				// Create the new assignment
				DiscreteProbabilityAssignment assignment = new DiscreteProbabilityAssignment();
				assignment.featurepairs = new FeatureValuePair[1];
				assignment.featurepairs[0] = new FeatureValuePair();
				assignment.featurepairs[0].featlabel = feature.featlabel;
				assignment.featurepairs[0].featvalue = featvalue;
				assignment.prob = getIndependentProb(feature.featlabel, featvalue);
				log("Independent probability: " + assignment.prob);

				// Add the assignment to the array
				assignments[i] = assignment;
			}
			return assignments;
		}
		else {
			log("ERROR: alternative values in feature are not specified");
			return null;
		}
	}

	
	/**
	 * Helper method to drastically reduce the distribution of unions containing > 1 proxies,
	 * and which turn out to have a prior distribution equal to the product of the prior 
	 * distribution of its included proxies
	 * 
	 * This is used to ensure that unions are only created if they have a prior distribution
	 * significantly higher than the prior distributions of their proxies -- that is, if they exhibit
	 * a strong internal coherence.
	 * 
	 * @param distrib the prior distribution to filter
	 * @param entity the entity
	 * @return
	 */
	private DiscreteProbabilityDistribution filterDistribution 
	(DiscreteProbabilityDistribution distrib, DiscreteProbabilityDistribution[] subdistribs) {


		// Compute the maximum probability for the prior distribution of the union
		float maxUnion = MaximumSearch.getMaximum(distrib);
		
		// And compute the product of the maximum probabilities for the prior distributions
		// of the included proxies
		float maxSubdistribs = 1.0f;
		for (int i = 0; i < subdistribs.length ; i++) {
			float maxSubdistrib = MaximumSearch.getMaximum(subdistribs[i]);
			maxSubdistribs = maxSubdistribs * maxSubdistrib;
		} 

		// In case the maximum for the union prior distribution turns out to be moreless equal to the 
		// product of the maximums for the proxy prior distributions, reduce the probabilities 
		// of the prior union distribution
		if (maxUnion < (maxSubdistribs + maxSubdistribs/5)) {
			distrib = ProbabilityUtils.multiplyDistributionWithConstantValue(distrib, 0.0000005f);
		}	

		return distrib;
	}
	
	
	/**
	 * Return the Bayesian network
	 */
	public BayesianNetwork getBayesianNetwork() {
		return network.getBayesianNetwork();
	}


	/**
	 * Logging facility
	 * @param s 
	 */

	private void log(String s) {
		if (LOGGING) {
			logger.debug("[BayesianNetworkManager] " + s);
		}
	}
}
