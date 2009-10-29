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

package binder.filtering;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.Vector;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.bayesiannetwork.BayesianNetworkWrapper;
import binder.utils.ProbabilityUtils;


/**
 * Configuration filter: utility library for ranking/filtering/selection 
 * union configurations
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 10/09/2009)
 */

public class ConfigurationFilter {

	// flag to activate error logging
	public static boolean ERRLOGGING = true;
	
	// flag to activate logging
	public static boolean LOGGING = true;

	
	private static Logger logger = ComponentLogger.getLogger(ConfigurationFilter.class);

	public static float ORPHAN_PROXY_PRIOR_PROB = 1.0f;
	
	// ===================================================================
	// METHODS FOR COMPUTATION & NORMALISATION OF CONFIG PROBABILITIES   
	// =================================================================== 


	/**
	 * Compute confidence scores for union configurations, normalise them, and set the results
	 * in their respective configProb fields
	 * 
	 * TODO: find a way to overcome the very small confidence score to scale up to hundreds of proxies
	 * 
	 * @param configs
	 */
	public static void computeConfidenceScoresForUnionConfigurations
	(Vector<UnionConfiguration> configs) {

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

			double score = 1.0f;
			for (int i = 0; i < config.includedUnions.length ; i++) {	
				Union union = config.includedUnions[i];
				float val;
				if (union.probExists > 0.0f) {
					val = union.probExists;
				}
				else  {
					val = getProbabilitiesSum (union.distribution);
				}
				score = score * val;
				
			} 
			
			if (config.orphanProxies != null) {
				for (int i = 0 ; i < config.orphanProxies.length ; i++) {
					Proxy orphan = config.orphanProxies[i];
					float probNoUnionExists = (1.0f - orphan.probExists) * ORPHAN_PROXY_PRIOR_PROB ;
					score = score * probNoUnionExists;
				}
			}
			config.configProb = score;
		}
		
		// normalise the results
		normaliseConfigProbabilities (configs);
	}
	

	/**
	 * Get the sum of all probability assignments a given distribution
	 * 
	 * @param distrib the probability distribution
	 * @return the probability value
	 */

	public static float getProbabilitiesSum (ProbabilityDistribution distrib) {

		// Case 1: distribution is discrete
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getProbabilitiesSum((DiscreteProbabilityDistribution) distrib);
		}

		// Case 2: distribution is a combined one
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return getProbabilitiesSum((CombinedProbabilityDistribution) distrib);
		}

		// and, "Houston we have a problem" 
		else {
			errlog("Sorry, only discrete or combined feature distributions are handled right now");
			log("Used class: " + distrib.getClass());
			return 0.0f;
		}
	}
	
	
	private static float getProbabilitiesSum (DiscreteProbabilityDistribution distrib) {
				
		float sum = 0.0f;
		for  (int i = 0 ; i < distrib.assignments.length ; i++) {
			sum += distrib.assignments[i].prob;
		}
		return sum;
	}
	
	
	
	

	/**
	 * Get the maximum probability value for a given combined distribution
	 * 
	 * @param distrib the combined probability distribution
	 * @return the probability value
	 */

	public static float getProbabilitiesSum (CombinedProbabilityDistribution distrib) {

		float sum = 0.0f;

		/** Here, we assume for the moment that the (1) the first distribution inside the combined 
		 * distribution is discrete, and (2) that it contains the complete list of possible assignments 
		 * for the combined distribution 
		 * TODO: find a nicer specification for the assignment in the combined distribution */

		if (distrib.distributions[0] instanceof DiscreteProbabilityDistribution) {
			DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];

			// loop on the possible assignments
			for (int i = 0; i < firstDistrib.assignments.length; i++) {

				DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];

				// Extract the probability value for the assignment
				float probValue = ProbabilityUtils.getProbabilityValue(distrib, assignment);		
				sum += probValue;
			}
		}
		else {
			errlog("WARNING: first distribution insided the combined distribution is not discrete!");
		}

		return sum;
	}

	
	
	
	
	
	/**
	 * Normalise the probabilities of the union configurations (in order to have a sum = 1)
	 * 
	 * @param configs the union configurations
	 */
	public static void normaliseConfigProbabilities (Vector<UnionConfiguration> configs) {

		// computes the sum of the probabibilities
		double sum = 0.0f;
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();
			sum += config.configProb;
		}

		// set the normalisation factor
		double alpha = 1.0f / sum;

		// apply the normalisation factor to all probabilities
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();
			config.configProb = alpha * config.configProb;
		}		 
	}
	
	/**
	 * Check whether the configuration probabilities are already computed or not
	 * @param configs
	 * @return true if already computed, false otherwise
	 */
	public static boolean areConfigProbsAlreadyComputed 
		(Vector<UnionConfiguration> configs) {
		
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();
			
			if (config.configProb == -1.0f) {
				return false;
			}
		}
		
		return true;
	}
	
	// ===================================================================
	// NBESTS FILTERING METHODS 
	// =================================================================== 

	
	/**
	 * Extract the NBests configurations amongst the complete set of union configurations.  
	 * The exact number of configurations to extract is set by the nb_nbests parameter
	 * 
	 * @param configs the union configurations
	 * @param nb_nbests number of configurations to keep
	 * @return set of selected union configurations
	 */
	
	public static Vector<UnionConfiguration> getNBestUnionConfigurations
	(Vector<UnionConfiguration> configs, int nb_nbests) {

		// check if the configuration probabilities are already computed
		if (!areConfigProbsAlreadyComputed(configs)) {
			errlog("WARNING: probabilities for union configurations are not computed!");
		}

		// List of NBests configuration
		Vector<UnionConfiguration> nbestConfigs = new Vector<UnionConfiguration>();

		// the threshold is set to be the lowest configuration probability in the NBests set
		double threshold = 9999.0f;
		
		// loop on the union configurations
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();
		
			// if the number of current nbests hasn't reached the maximum number, simply 
			// add the configuration
			if (nbestConfigs.size() < nb_nbests) {

				nbestConfigs.add(config);

				// if the current config probability is lower than the current threshold, 
				// assign the probability to the threshold
				if (config.configProb < threshold) {
					threshold = config.configProb;
				}
			}

			// else, we check if the config probability is higher than the threshold
			else if (config.configProb > threshold) {
				
					// If it is, we extract the lowest-probability union configuration ...
					UnionConfiguration worstinNBests = getWorstUnionConfiguration(nbestConfigs);
					
					// ... and we remove it...
					nbestConfigs.remove(worstinNBests);
					
					// ... to replace it by the new configuration
					nbestConfigs.add(config);
					
					// Finally, we search for the second lowest-probability configuration, and
					// assign the threshold to be its configuration probability
					UnionConfiguration secondworst = getWorstUnionConfiguration(nbestConfigs);
					threshold = secondworst.configProb;
				}
		}

		return nbestConfigs;
	}
	
	
	// ===================================================================
	// METHODS FOR BEST AND WORST UNION SELECTION OF CONFIGURATIONS
	// =================================================================== 



	/**
	 * Extract the worst (lowest-probability) union configuration out of the configs set
	 * 
	 * @param configs the union configurations
	 * @return the lowest-probability configuration
	 */
	
	public static UnionConfiguration getWorstUnionConfiguration(Vector<UnionConfiguration> configs) {

		double threshold = 99999.0f;
		UnionConfiguration worstConfig = null;

		// loop on the union configurations
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

				// if the current probability is lower than the threshold, reassign the threshold
				// and the worst config
				if (config.configProb < threshold) {
					threshold = config.configProb;
					worstConfig = config;
				}
		}

		return worstConfig;
	}



	/**
	 * Extract the best (highest-probability) union configuration out of the 
	 * AlternativeUnionConfigurations object
	 * 
	 * @param configs the union AlternativeUnionConfigurations object
	 * @return the highest-probability configuration
	 */
	
	public static UnionConfiguration getBestUnionConfiguration
		(AlternativeUnionConfigurations configs) {
		
		// creates a vector of unions configurations
		Vector<UnionConfiguration> unionconfigsV = new Vector<UnionConfiguration>();
		for (int i = 0 ; i < configs.alterconfigs.length ; i++) {
			unionconfigsV.add(configs.alterconfigs[i]);
		}
		
		// and extract the best union configuration from it
		return getBestUnionConfiguration (unionconfigsV);
	}

	
	
	
	

	/**
	 * Extract the union configuration of rank n out of the 
	 * AlternativeUnionConfigurations object
	 * 
	 * @param configs the union AlternativeUnionConfigurations object
	 * @return the configuration of rank n
	 */
	
	public static UnionConfiguration getUnionConfigurationOfRankN
		(AlternativeUnionConfigurations configs, int rank) {
		
		if (configs != null && configs.alterconfigs != null) {
		// creates a vector of unions configurations
		Vector<UnionConfiguration> unionconfigsV = new Vector<UnionConfiguration>();
		for (int i = 0 ; i < configs.alterconfigs.length ; i++) {
			unionconfigsV.add(configs.alterconfigs[i]);
		}
		
		// and extract the best union configuration from it
		Vector<UnionConfiguration> nbests = getNBestUnionConfigurations (unionconfigsV, rank);
		UnionConfiguration [] nbestsArray = new UnionConfiguration[nbests.size()];
		nbestsArray = nbests.toArray(nbestsArray);
				
		Comparator<UnionConfiguration> comparator = new ConfigurationComparator();
		
		Arrays.sort(nbestsArray, comparator);
		
		return nbestsArray[nbestsArray.length - rank];
		}
		else {
			return null;
		}
	}
	

	/**
	 * Extract the best (highest-probability) union configuration out of the configs set
	 * 
	 * @param configs the union configurations
	 * @return the highest-probability configuration
	 */
	
	public static UnionConfiguration getBestUnionConfiguration(Vector<UnionConfiguration> configs) {

		double maxAverage = -1.0f;
		UnionConfiguration bestConfig = null;

		// loop on the union configurations
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

			// if the current probability is higher than the threshold, reassign the threshold
			// and the best config
			if (config.configProb > maxAverage) {
				maxAverage = config.configProb;
				bestConfig = config;
			}
		}

		return bestConfig;
	}


	
	// ===================================================================
	// UTILITY METHODS 
	// =================================================================== 



	public static void errlog(String s) {
		if (ERRLOGGING)
			logger.debug("[ConfigFilter] " + s);
	}
	
	
	public static void log(String s) {
		if (LOGGING)
			logger.debug("[ConfigFilter] " + s);
	}

}
