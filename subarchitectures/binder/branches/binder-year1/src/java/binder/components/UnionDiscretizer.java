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

package binder.components;

import java.util.Map;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.filtering.ConfigurationFilter;
import binder.filtering.EntityFilter;
import binder.utils.BinderUtils;
import binder.utils.FeatureValueUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


/**
 * Discretization mechanism for the binder: takes as input the whole set of alternative union 
 * configurations (i.e. the probability distribution over possible unions), and outputs
 * a single best union configuration.  This union configuration which is selected is the 
 * one having the highest probability value.  
 * 
 * In other words, the discretizer collapses the union distributions onto a set of high-probability, 
 * single point representations, which can be then easily reused to generate particular symbolic
 * states and exploited for deliberative reasoning or learning.
 *
 * @author Pierre Lison
 * @version 22/09/2009 (started 01/09/2009)
 *
 */

public class UnionDiscretizer extends ManagedComponent {

	// whether to consider only maximum feature values in feature or not
	private static boolean onlyMaxFeatureValues = false;
	
	// flag to activate error logging
	public static boolean ERRLOGGING = true;

	
	/**
	 * Initialisation - add a change filter for AlternativeUnionConfigurations on the binder WM
	 */
	@Override
	public void start() {

		// if the set of possible union configurations has been updated, update the
		// monitor accordingly
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AlternativeUnionConfigurations.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					if (!_wmc.src.equals("bmmonitor")) {
						log("Source: " + _wmc.src);
					AlternativeUnionConfigurations alterconfigs = 
						getMemoryEntry(_wmc.address, AlternativeUnionConfigurations.class);

					UnionConfiguration uc = extractBestUnionConfiguration(alterconfigs);

					addBestUnionConfigurationToWM(uc);
					} 

				}
				catch (Exception e) {
					e.printStackTrace();
				}
				}
		});
	}

	
	
	/**
	 * Set configuration parameters
	 */

	@Override
	public void configure(Map<String, String> _config) {
		
		if (_config.containsKey("--onlymaxfeatvalues")) {
			onlyMaxFeatureValues = Boolean.parseBoolean(_config.get("--onlymaxfeatvalues"));
		} 
	}

	
	/**
	 * Extract the single best union configuration from the set of possible ones specified in
	 * the AlternativeUnionConfigurations object.
	 * 
	 * @param alterconfigs the alternative union configurations
	 * @return the single best (highest-probability) union configuration
	 */
	
	synchronized public UnionConfiguration extractBestUnionConfiguration 
	(AlternativeUnionConfigurations alterconfigs) {

		log("--------START DISCRETISATION ----------");
		long initTime = System.currentTimeMillis();


		log("Number of alternative union configurations: "  + alterconfigs.alterconfigs.length);
		
		// Compute the best union configuration out of the possible ones
		UnionConfiguration bestConfiguration = 
			ConfigurationFilter.getBestUnionConfiguration(alterconfigs);

		Vector<Union> unions = new Vector<Union>();

		UnionConfiguration discretizedConfig;
		if (bestConfiguration != null) {
		log("Best union configuration successfully computed");
		log("Number of unions in selected configuration: " + 
				bestConfiguration.includedUnions.length);

		
		// In the chosen union configuration, loop on the included unions, and compute
		// for each of them the instance with the maximum probability
		for (int i = 0 ; i < bestConfiguration.includedUnions.length ; i++) {
			Union union = bestConfiguration.includedUnions[i];
			// If only maximum-probability values are allowed, compute a new union with only these
			if (onlyMaxFeatureValues) {
				union = EntityFilter.getUnionWithMaximumProbability(union);
			}
			
			unions.add(union);
		} 
		
		// Create a new configuration with the updated unions
		Union[] includedUnions = new Union[unions.size()];
		includedUnions = unions.toArray(includedUnions);
		Proxy[] orphanProxies = bestConfiguration.orphanProxies;
		double configProb = bestConfiguration.configProb;
		discretizedConfig = new UnionConfiguration(includedUnions, orphanProxies, configProb);

		}
		else {
			errlog("WARNING: no best union configuration could be found! (returned null)");
			discretizedConfig = new UnionConfiguration(new Union[0], new Proxy[0], 0.0f);
		}
		
		long finalTime = System.currentTimeMillis();
		log("Total discretisation time: " + (finalTime - initTime)/1000.0 + " seconds");
		log("--------STOP DISCRETISATION ----------");

		return discretizedConfig;
	}

	

	/**
	 * Extract the union configuration of rank n from the set of possible ones specified in
	 * the AlternativeUnionConfigurations object.
	 * 
	 * @param alterconfigs the alternative union configurations
	 * @param rank the rank of the configuration (1 being the rank with the highest score, 
	 *             2 the rank for the second-highest score, etc.)
	 * @return the single best (highest-probability) union configuration
	 */
	
	synchronized public UnionConfiguration extractUnionConfigurationOfRankN 
	(AlternativeUnionConfigurations alterconfigs, int rank) {

		log("--------START DISCRETISATION ----------");
		long initTime = System.currentTimeMillis();
 
		log("Number of alternative union configurations: "  + alterconfigs.alterconfigs.length);
		
		// Compute the best union configuration out of the possible ones
		UnionConfiguration rankNConfig = 
			ConfigurationFilter.getUnionConfigurationOfRankN(alterconfigs, rank);

		Vector<Union> unions = new Vector<Union>();

		UnionConfiguration discretizedConfig;
		
		if (rankNConfig != null) {
		log("Rank n union configuration successfully computed");
		log("Number of unions in selected configuration: " + 
				rankNConfig.includedUnions.length);

		
		// In the chosen union configuration, loop on the included unions, and compute
		// for each of them the instance with the maximum probability
		for (int i = 0 ; i < rankNConfig.includedUnions.length ; i++) {
			Union union = rankNConfig.includedUnions[i];
			// If only maximum-probability values are allowed, compute a new union with only these
			
			if (onlyMaxFeatureValues) {
				union = EntityFilter.getUnionWithMaximumProbability(union);
			}
			
			unions.add(union);
		} 
		
		// Create a new configuration with the updated unions
		Union[] includedUnions = new Union[unions.size()];
		includedUnions = unions.toArray(includedUnions);
		Proxy[] orphanProxies = rankNConfig.orphanProxies;
		double configProb = rankNConfig.configProb;

		discretizedConfig = new UnionConfiguration(includedUnions, orphanProxies, configProb);

		}
		else {
			errlog("WARNING: no best union configuration could be found! (returned null)");
			discretizedConfig = new UnionConfiguration(new Union[0], new Proxy[0], 0.0f);
		}
		
		long finalTime = System.currentTimeMillis();
		log("Total discretisation time: " + (finalTime - initTime)/1000.0 + " seconds");
		log("--------STOP DISCRETISATION ----------");

		return discretizedConfig;
	}
	
	
	/**
	 * Add union configuration to the binder working memory
	 * 
	 * @param config the union configuration
	 */
	protected void addBestUnionConfigurationToWM(UnionConfiguration config) {

		try {
			CASTData<UnionConfiguration>[] existingconfigs = 
				getWorkingMemoryEntries(Binder.BINDER_SA, UnionConfiguration.class);
			if (existingconfigs != null && existingconfigs.length > 0) {
				overwriteWorkingMemory (existingconfigs[0].getID(), Binder.BINDER_SA, config);
			}
			else {				
				addToWorkingMemory(newDataID(), Binder.BINDER_SA, config);
			}
			log("Union configuration succesfully added/updated");

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	

	private static void errlog (String s) {
		if (ERRLOGGING)
		System.out.println("[UnionConstructor] " + s);
	}

}
