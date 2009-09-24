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

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;


/**
 * Configuration filter: utility library for ranking/filtering/selection 
 * union configurations
 * 
 * @author Pierre Lison
 * @version 23/09/2009
 * @started 10/09/2009
 */

public class ConfigurationFilter {

	// flag to activate error logging
	public static boolean ERRLOGGING = true;
	
	// flag to activate logging
	public static boolean LOGGING = false;

	

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

			double multiplication = 1.0f;
			for (int i = 0; i < config.includedUnions.length ; i++) {	
				Union union = config.includedUnions[i];
				float max = MaximumSearch.getMaximum(union);
				multiplication = multiplication * max;
				
			} 
			
			config.configProb = multiplication;
			log("configProb: " + config.configProb);
		}
		
		// normalise the results
		normaliseConfigProbabilities (configs);
	}
	

	
	/**
	 * Normalise the probabilities of the union configurations (in order to have a sum = 1)
	 * 
	 * @param configs the union configurations
	 */
	private static void normaliseConfigProbabilities (Vector<UnionConfiguration> configs) {

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
	 * @return
	 */
	public static boolean areConfigProbsAlreadyComputed 
		(Vector<UnionConfiguration> configs) {
		
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();
			
			if (config.configProb == 0.0f) {
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
		System.out.println("[GradientDescent] " + s);
	}
	
	
	public static void log(String s) {
		if (LOGGING)
		System.out.println("[GradientDescent] " + s);
	}

}
