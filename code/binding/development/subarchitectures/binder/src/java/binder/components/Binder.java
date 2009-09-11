
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

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.specialentities.RelationUnion;
import binder.utils.BinderUtils;
import binder.utils.GradientDescent;
import binder.utils.ProbabilityUtils;
import binder.utils.UnionConstructor;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

/**
 * The core binding algorithm - it takes proxies as inputs, and generate
 * a distribution of possible union configurations (sequences of unions)
 * for the proxies, given a predefined Bayesian Network specifying
 * the correlations between features
 * 
 * @author Pierre Lison
 * @version 09/09/2009
 * @started 01/07/2009
 */

public class Binder extends ManagedComponent  {


	// the union constructor
	private UnionConstructor constructor;

	// whether to perform incremental or full rebinding
	private boolean incrementalBinding = true;

	// whether to add unknown values to each feature
	private boolean addUnknowns = true;
	
	// TO IMPLEMENT / TEST
	private boolean normaliseDistributions = false;
	
	// Filtering parameters: maximum number of union configurations
	// to keep in the binder at a given time
	private int NB_CONFIGURATIONS_TO_KEEP = 1;

	// The union configurations computed for the current state 
	// of the binder WM (modulo filtering)
	Vector<UnionConfiguration> currentUnionConfigurations ;

	
	

	// ================================================================= 
	// INITIALISATION METHODS                   
	// ================================================================= 


	/**
	 *  Construct a new binder
	 */
	public Binder() {
		constructor = new UnionConstructor();
	}


	/** 
	 * Add filters on proxy insertions, modifications (overwrite) and deletions,
	 * and initialise the binder
	 * 
	 */
	@Override
	public void start() {

		// Proxy insertion
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Proxy.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					proxyInsertion(_wmc);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});

		// Proxy modification
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Proxy.class,
				WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					proxyUpdate(_wmc);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});

		// Proxy deletion
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Proxy.class,
				WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					proxyDeletion(_wmc);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});

		// Initialisation stuff
		initializeUnionConfigurations();

		log("Binding Monitor successfully started");
	}


	/**
	 * (re)initialize the binder with a single union configuration
	 */

	private void initializeUnionConfigurations () {
		currentUnionConfigurations = new Vector<UnionConfiguration>();
		UnionConfiguration initialConfig = new UnionConfiguration();
		initialConfig.includedUnions = new Union[0];
		currentUnionConfigurations.add(initialConfig);
	}



	/**
	 * Set configuration parameters
	 */

	@Override
	public void configure(Map<String, String> _config) {
		if (_config.containsKey("--alpha")) {
			constructor.setAlphaParam(Float.parseFloat(_config.get("--alpha")));
		} 
		if (_config.containsKey("--incremental")) {
			incrementalBinding = Boolean.parseBoolean(_config.get("--incremental"));
		} 
	}


	// ================================================================= 
	// METHODS FOR UPDATING THE BINDING WORKING MEMORY   
	// ================================================================= 


	/**
	 * Update the binding working memory after the overwrite (modification) of 
	 * an existing proxy
	 * 
	 * @param wmc the working memory change triggering the update
	 * @pre currentUnionConfigurations contains the current set of union 
	 *      configurations in the WM
	 * @post update of currentUnionConfigurations with the proxy change
	 */

	private void proxyUpdate (WorkingMemoryChange wmc) {

		log("--------START BINDING UPDATE ----------");
		log(" TRIGGERED BY: overwrite of existing proxy ");

		try {
			// The proxy which was modified
			Proxy updatedProxy= getMemoryEntry(wmc.address, Proxy.class);

			updatedProxy = BinderUtils.completeProxy(updatedProxy, addUnknowns);
			updatedProxy.timeStamp = System.currentTimeMillis();;

			// Loop on the current union configurations to update each of them in turn
			for (Enumeration<UnionConfiguration> configs = 
				currentUnionConfigurations.elements() ; configs.hasMoreElements(); ) {

				UnionConfiguration existingUnionConfig = configs.nextElement();				

				// Loop on each union in the configuration
				for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {

					Union existingUnion = existingUnionConfig.includedUnions[i];

					// Loop on each proxy included in the union
					for (int j = 0; j < existingUnion.includedProxies.length ; j++) {

						Proxy existingProxy = existingUnion.includedProxies[j];

						// If the proxy turns out to coincide with the updated proxy, regenerate its union
						// and include it into the union configuration
						if (existingProxy.entityID.equals(updatedProxy.entityID)) {
							Vector<PerceivedEntity> proxies = 
								getOtherProxies(existingUnion.includedProxies, existingProxy);
							proxies.add(updatedProxy);
							Union updatedUnion = constructor.constructNewUnion(proxies, newDataID());								
							updatedUnion.entityID = existingUnion.entityID ;
							existingUnionConfig.includedUnions[i] = updatedUnion;
						}
					}
				}

			}

			// Update the alternative union configurations in the WM
			AlternativeUnionConfigurations alters = buildNewAlternativeUnionConfigurations();
			updateWM(alters); 

		}
		catch (Exception e) {
			log("Updated proxy cannot be found in the WM");
			e.printStackTrace();
		}

		log("--------STOP BINDING UPDATE----------");
	}


	
	/**
	 * Update the binding working memory after the deletion of an existing proxy
	 * 
	 * @param wmc the working memory change triggering the update
	 * @pre currentUnionConfigurations contains the current set of union \
	 *      configurations in the WM
	 * @post update of currentUnionConfigurations to accommodate the proxy deletion
	 */

	private void proxyDeletion (WorkingMemoryChange wmc) {

		log("--------START BINDING UPDATE ----------");
		log(" TRIGGERED BY: proxy deletion ");
		
		try {
			// The ID of the deleted proxy
			String deletedProxyID= wmc.address.id;

			// Loop on the current union configurations to update each of them in turn
			for (Enumeration<UnionConfiguration> configs = 
				currentUnionConfigurations.elements() ; configs.hasMoreElements(); ) {

				UnionConfiguration existingUnionConfig = configs.nextElement();				

				// Loop on the unions in the configuration
				for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {

					Union existingUnion = existingUnionConfig.includedUnions[i];

					// Loop on the proxies included in the unions
					for (int j = 0; j < existingUnion.includedProxies.length ; j++) {

						Proxy existingProxy = existingUnion.includedProxies[j];

						// If the proxy turns out to be the deleted proxy, update or remove
						// the union and the configuration
						if (existingProxy.entityID.equals(deletedProxyID)) {
							Vector<PerceivedEntity> proxies = 
								getOtherProxies(existingUnion.includedProxies, existingProxy);
							if (proxies.size() > 0) { 
								Union updatedUnion = constructor.constructNewUnion(proxies, newDataID());								
								updatedUnion.entityID = existingUnion.entityID ;
								existingUnionConfig.includedUnions[i] = updatedUnion;
							}
							else {
								existingUnionConfig = 
									removeUnionFromConfig(existingUnionConfig, existingUnion);
							}
						}
					}
				}

			}

			// Update the alternative union configurations
			AlternativeUnionConfigurations alters = buildNewAlternativeUnionConfigurations();
			updateWM(alters); 

		}
		catch (Exception e) {
			e.printStackTrace();
		}

		log("--------STOP BINDING UPDATE (AFTER DELETION) ----------");
	}


	/**
	 * Update the binding working memory after the insertion of a new proxy
	 * 
	 * @param wmc the working memory change triggering the update
	 * @pre   currentUnionConfigurations contains the current set of union \
	 *        configurations in the WM
	 * @post  update of currentUnionConfigurations to accommodate the new 
	 *        proxy insertion
	 */

	private void proxyInsertion(WorkingMemoryChange wmc) {
		log("--------START BINDING UPDATE ----------");
	
		long initTime = System.currentTimeMillis();

		try {
			// Extract the new proxy
			Proxy newProxy = getMemoryEntry(wmc.address, Proxy.class);

			newProxy.timeStamp = System.currentTimeMillis();;

			log(" TRIGGERED BY: insertion of new proxy " + newProxy.entityID +
					" (" + newProxy.getClass().getSimpleName() + ") ");
		
			newProxy = BinderUtils.completeProxy(newProxy, addUnknowns);

			// Perform the binding (either incrementally or by full rebinding)
			if (incrementalBinding) {
				incrementalBinding(newProxy);
			}
			else {
				fullRebinding();
			}

		}
		catch (Exception e) {
			e.printStackTrace();
		}

		long finalTime = System.currentTimeMillis();
		log("Total binding time: " + (finalTime - initTime)/1000.0 + " seconds");
		log("--------STOP BINDING UPDATE----------");

	}


	// ================================================================= 
	// INCREMENTAL AND FULL BINDING METHODS   
	// ================================================================= 


	/**
	 * Restart the binding process from the start, by removing all existing unions
	 * and reconstructing the unions one by one
	 */

	private void fullRebinding() {
		try {
			log("Perform full rebinding...");

			initializeUnionConfigurations();

			CASTData<Proxy>[] proxies = getWorkingMemoryEntries(Proxy.class);
			for (int i = 0 ; i < proxies.length; i++) {
				incrementalBinding(proxies[i].getData());
			}

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	
	/**
	 * Update the binding working memory by recomputing the union configurations
	 * after the insertion of a new proxy
	 * @param newProxy the newly inserted proxy
	 * 
	 */
	private void incrementalBinding(Proxy newProxy) {
		try {
			log("--> Perform incremental binding");

			log("Constructing initial union...");
			// Construct the initial union (containing only the new proxy)
			Union newUnion = constructor.getInitialUnion(newProxy, newDataID());

			log("Construction of initial union finished, moving to unions of more than 1 proxy...");

			log("Number of current configurations: "  + currentUnionConfigurations.size());
			
			// The new union configurations
			Vector<UnionConfiguration> newUnionConfigurations = new Vector<UnionConfiguration>();
			
			// List of unions already computed and merged
			HashMap<String,Union> alreadyMergedUnions = new HashMap<String, Union>();

			// loop on the current union configurations
			for (Enumeration<UnionConfiguration> configs = 
				currentUnionConfigurations.elements() ; configs.hasMoreElements(); ) {

				UnionConfiguration existingUnionConfig = configs.nextElement();				

				// Create and add a new configuration containing the single-proxy union
				UnionConfiguration newConfigWithSingleUnion = 
					createNewUnionConfiguration (existingUnionConfig, newUnion);
				newUnionConfigurations.add(newConfigWithSingleUnion);	

				// Loop on the unions in the union configuration
				for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {

					Union existingUnion = existingUnionConfig.includedUnions[i];

					// A new, several-proxies union
					Union newMergedUnion;
					
					// check if the current union and the new one can be merged
					if (!hasConflicts(existingUnion, newUnion)) {

						// Check if the union has already been constructed
						if (!alreadyMergedUnions.containsKey(existingUnion.entityID)) {
							
							// If not, construct the merged union
							Vector<PerceivedEntity> unionsToMerge = new Vector<PerceivedEntity>();
							unionsToMerge.add(existingUnion);
							unionsToMerge.add(newUnion);
							newMergedUnion = constructor.constructNewUnion(unionsToMerge, newDataID());
							alreadyMergedUnions.put(existingUnion.entityID, newMergedUnion);
						}
						// or simply fetch the already computed union
						else {
							newMergedUnion = alreadyMergedUnions.get(existingUnion.entityID);
						}
						// create and add a new union configuration with the new merged union
						UnionConfiguration newConfigWithMergedUnion = 
							createNewUnionConfiguration (existingUnionConfig, newMergedUnion, existingUnion);
						newUnionConfigurations.add(newConfigWithMergedUnion);

					}
				}
			}

			// Get the nbest configurations (with N as a parameter)
			Vector<UnionConfiguration> NBests = 
				GradientDescent.getNBestUnionConfigurations
				(newUnionConfigurations, NB_CONFIGURATIONS_TO_KEEP);

			// update the list of possible unions
			currentUnionConfigurations = NBests;

			log("Total number of union configurations generated: " + currentUnionConfigurations.size());

			// Add everything to the working memory
			AlternativeUnionConfigurations alters = buildNewAlternativeUnionConfigurations();
			updateWM(alters); 

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}


	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 


	/**
	 * Get a vector containing all elements in the "proxies" array apart from proxyToExclude
	 * @param proxies the initial array of proxies
	 * @param proxyToExclude the proxy to exclude
	 * @return vector of PerceivedEntities
	 */
	private Vector<PerceivedEntity> getOtherProxies (Proxy[] proxies, Proxy proxyToExclude) {

		Vector<PerceivedEntity> otherProxies = new Vector<PerceivedEntity>();

		for (int i = 0; i < proxies.length ; i++) {
			Proxy prox = proxies[i];
			if (!prox.equals(proxyToExclude)) {
				otherProxies.add(prox);
			}
		}
		return otherProxies;
	}



	/**
	 * Remove the union from the configuration
	 * @param existingconfig
	 * @param unionToDelete
	 * @return
	 */
	private UnionConfiguration removeUnionFromConfig
	(UnionConfiguration existingconfig, Union unionToDelete) {

		Vector<Union> unionsInConfig = new Vector<Union>();
		for (int i = 0; i < existingconfig.includedUnions.length ; i++) {
			Union curUnion = existingconfig.includedUnions[i];
			if (!curUnion.equals(unionToDelete)) {
				unionsInConfig.add(curUnion);
			}
		}
		existingconfig.includedUnions = new Union[unionsInConfig.size()];
		existingconfig.includedUnions = unionsInConfig.toArray(existingconfig.includedUnions);

		return existingconfig;
	}


	/**
	 * Build an AlternativeUnionConfigurations containing all configurations listed in
	 * currentUnionConfigurations
	 * @return
	 */
	private AlternativeUnionConfigurations buildNewAlternativeUnionConfigurations () {

		AlternativeUnionConfigurations alters = new AlternativeUnionConfigurations();
		alters.alterconfigs = new UnionConfiguration[currentUnionConfigurations.size()];
		for (int i = 0; i < alters.alterconfigs.length ; i++) {
			alters.alterconfigs[i] = currentUnionConfigurations.elementAt(i); 
		}
		return alters;
	}


	/**
	 * Create a new union configuration based on an existing one and a new union 
	 * to add
	 * @param existingUnionConfig
	 * @param unionToAdd
	 * @return
	 */
	private UnionConfiguration createNewUnionConfiguration
	(UnionConfiguration existingUnionConfig, Union unionToAdd) {			
		return createNewUnionConfiguration(existingUnionConfig, unionToAdd, new Vector<Union>());
	}

	
	/**
	 * Create a new union configuration based on an existing configuration, a union
	 * to add, and an union to remove
	 * @param existingUnionConfig
	 * @param unionToAdd
	 * @param unionToRemove
	 * @return
	 */
	private UnionConfiguration createNewUnionConfiguration
	(UnionConfiguration existingUnionConfig, Union unionToAdd, Union unionToRemove) {

		Vector<Union> unionsToRemove = new Vector<Union>();
		unionsToRemove.add(unionToRemove);
		return createNewUnionConfiguration(existingUnionConfig, unionToAdd, unionsToRemove);
	}



	/**
	 * Create a new union configuration based on an existing configuration, a new
	 * union to add, and a list of unions to remove
	 * @param existingUnionConfig
	 * @param unionToAdd
	 * @param unionsToRemove
	 * @return
	 */
	private UnionConfiguration createNewUnionConfiguration(UnionConfiguration existingUnionConfig, 
			Union unionToAdd, Vector<Union> unionsToRemove) {

		UnionConfiguration newConfig = new UnionConfiguration();
		int nbUnions = existingUnionConfig.includedUnions.length + 1 - unionsToRemove.size();
		newConfig.includedUnions = new Union[nbUnions];

		int count = 0;
		for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {
			if (!unionsToRemove.contains(existingUnionConfig.includedUnions[i])) {
				newConfig.includedUnions[i- count] = existingUnionConfig.includedUnions[i];
			}
			else {
				count ++;
			}
		}

		newConfig.includedUnions[nbUnions - 1] = unionToAdd;

		return newConfig;

	}

	/**
	 * Check if the two unions are from different originating subarchitectures
	 * @param union1
	 * @param union2
	 * @return
	 */
	private boolean hasConflicts(Union union1, Union union2) {	
		for (int i = 0 ; i < union1.includedProxies.length ; i++) {
			Proxy proxyi = union1.includedProxies[i];
			for (int j = 0 ; j < union2.includedProxies.length ; j++) {
				Proxy proxyj = union2.includedProxies[j];
				if (proxyi.origin.subarchId.equals(proxyj.origin.subarchId)) {
					return true;
				}
			}
		}	
		if (union1 instanceof RelationUnion && 
				! (union2 instanceof RelationUnion)) {
			return true;
		}
		if (! (union1 instanceof RelationUnion) && 
				union2 instanceof RelationUnion) {
			return true;
		}
		
		return false;
	}

	/**
	 * Update the binder working memory with a new AlternativeUnionConfigurations
	 * (add if none exists, else overwrite existing one)
	 * @param configs
	 */
	private void updateWM(AlternativeUnionConfigurations configs) {

		try {

			CASTData<AlternativeUnionConfigurations>[] alterconfigs = 
				getWorkingMemoryEntries(AlternativeUnionConfigurations.class);

			if (alterconfigs.length == 0) {
				addToWorkingMemory(newDataID(), configs);
			}
			else {
				overwriteWorkingMemory(alterconfigs[0].getID(), configs);
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

}
