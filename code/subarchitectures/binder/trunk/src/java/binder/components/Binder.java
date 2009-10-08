
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
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.RelationUnion;
import binder.utils.BinderUtils;
import binder.filtering.ConfigurationFilter;
import binder.utils.DistributionGeneration;
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
 * @version $Date$ (started 01/07/2009)
 */

public class Binder extends ManagedComponent  {
 

	// the union constructor
	private UnionConstructor constructor;

	// whether to perform incremental or full rebinding
	private boolean incrementalBinding = true;

	// whether to add unknown values to each feature
	private boolean addUnknowns = true;

	// Text specification of the bay
	private String bayesianNetworkConfigFile = "./subarchitectures/binder/config/bayesiannetwork.txt";

	// Filtering parameters: 
	
	// maximum number of union configurations
	// to keep in the binder at a given time
	private int nbestsFilter = 5;
	
	// maximum number of assignments in proxy distribution
	// to keep in each proxy (0 means no filtering)
	static public int proxyDistribFilter = 10;


	private boolean normaliseDistributions = true;
	
	// The union configurations computed for the current state 
	// of the binder WM (modulo filtering)
	private Vector<UnionConfiguration> currentUnionConfigurations ;

	
	public static String BINDER_SA = "binder";

	
	// flag to activate error logging
	public static boolean ERRLOGGING = true;



	// ================================================================= 
	// INITIALISATION METHODS                   
	// ================================================================= 


	/**
	 *  Construct a new binder
	 */
	public Binder() {	}


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

		BINDER_SA = this.getSubarchitectureID();
		
		log("Binder successfully started");
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

		if (_config.containsKey("--bayesiannetworkfile")) {
			bayesianNetworkConfigFile = _config.get("--bayesiannetworkfile");
		} 
		constructor = new UnionConstructor(bayesianNetworkConfigFile);

		if (_config.containsKey("--alpha")) {
			constructor.setAlphaParam(Float.parseFloat(_config.get("--alpha")));
		} 
		if (_config.containsKey("--incremental")) {
			incrementalBinding = Boolean.parseBoolean(_config.get("--incremental"));
		}

		if (_config.containsKey("--addunknowns")) {
			addUnknowns = Boolean.parseBoolean(_config.get("--addunknowns"));
		}

		if (_config.containsKey("--normalise")) {
			normaliseDistributions = Boolean.parseBoolean(_config.get("--normalise"));
		}

		if (_config.containsKey("--nbestsfilter")) {
			nbestsFilter = Integer.parseInt(_config.get("--nbestsfilter"));
		}
		if (_config.containsKey("--proxydistribfilter")) {
			proxyDistribFilter = Integer.parseInt(_config.get("--proxydistribfilter"));
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
		log("TRIGGERED BY: overwrite of existing proxy ");

		try {
			// The proxy which was modified
			Proxy updatedProxy= getMemoryEntry(wmc.address, Proxy.class);

			BinderUtils.completeProxy(updatedProxy, addUnknowns, proxyDistribFilter);
			updatedProxy.timeStamp = getCASTTime();

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
							Union updatedUnion = constructor.constructNewUnion(proxies, existingUnion.entityID, getCASTTime());								
							existingUnionConfig.includedUnions[i] = updatedUnion;
						}
					}
				}

			}

			recompute(currentUnionConfigurations);
			
			// Update the alternative union configurations in the WM
			AlternativeUnionConfigurations alters = 
				buildNewAlternativeUnionConfigurations(currentUnionConfigurations);
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
		log("TRIGGERED BY: proxy deletion ");

		try {
			// The ID of the deleted proxy
			String deletedProxyID= wmc.address.id;

			Vector<UnionConfiguration> newUnionConfigs = new Vector<UnionConfiguration>();

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
								
								Union updatedUnion = 
									constructor.constructNewUnion(proxies, existingUnion.entityID, getCASTTime());								
								existingUnionConfig.includedUnions[i] = updatedUnion;
							}
							else {
								existingUnionConfig = 
									removeUnionFromConfig(existingUnionConfig, existingUnion);
							}
							
							if (!isConfigurationAlreadyIncluded(currentUnionConfigurations, existingUnionConfig)) {
								newUnionConfigs.add(existingUnionConfig);
							}
						}
					}
				}

			}

			log("Total number of union configurations generated (before filtering): " + newUnionConfigs.size());

			currentUnionConfigurations = newUnionConfigs;
			
			recompute(currentUnionConfigurations);	

			
			// Update the alternative union configurations
			AlternativeUnionConfigurations alters = 
				buildNewAlternativeUnionConfigurations(currentUnionConfigurations);
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

			newProxy.timeStamp = getCASTTime();

			log("TRIGGERED BY: insertion of new proxy " + newProxy.entityID +
					" (" + newProxy.getClass().getSimpleName() + ") ");

			BinderUtils.completeProxy(newProxy, addUnknowns, proxyDistribFilter);

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
	 * Update the binding working memory by recomputing the union configurations
	 * after the insertion of a new proxy
	 * 
	 * TODO: add threshold to filter out proxies below a certain existence probability value
	 * 
	 * @param newProxy the newly inserted proxy
	 * 
	 */
	private void incrementalBinding(Proxy newProxy) {
		try {
			log("--> Perform incremental binding");

			log("Constructing initial union...");
			// Construct the initial union (containing only the new proxy)
			Union newUnion = constructor.constructInitialUnion(newProxy, newDataID(), getCASTTime());
			
			log("Construction of initial union finished, moving to unions of more than 1 proxy...");

			log("Number of current configurations: "  + currentUnionConfigurations.size());
 
			// The new union configurations
			Vector<UnionConfiguration> newUnionConfigs = new Vector<UnionConfiguration>();

			// List of unions already computed and merged
			HashMap<Union,Union> alreadyMergedUnions = new HashMap<Union, Union>();

			// loop on the current union configurations
			for (Enumeration<UnionConfiguration> configs = 
				currentUnionConfigurations.elements() ; configs.hasMoreElements(); ) {

				UnionConfiguration existingUnionConfig = configs.nextElement();				

				// Create and add a new configuration containing the single-proxy union
				UnionConfiguration newConfigWithSingleUnion = 
					createNewUnionConfiguration (existingUnionConfig, newUnion);
				newUnionConfigs.add(newConfigWithSingleUnion);	

				
				if (newUnion instanceof RelationUnion) {
					specifyUnionSourceAndTarget((RelationUnion)newUnion, existingUnionConfig);
				}
				
				// Loop on the unions in the union configuration
				for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {
					Union existingUnion = existingUnionConfig.includedUnions[i];

					// A new, several-proxies union
					Union newMergedUnion;

					// check if the current union and the new one can be merged
					if ( !hasConflicts(existingUnion, newUnion)) {

		//				log("constructing new union between "  + existingUnion.entityID + " and " + newUnion.entityID );
						if (!alreadyComputed(existingUnion, alreadyMergedUnions)) {

							// If not, construct the merged union
							Vector<PerceivedEntity> unionsToMerge = new Vector<PerceivedEntity>();
							unionsToMerge.add(existingUnion);
							unionsToMerge.add(newUnion);
							newMergedUnion = constructor.constructNewUnion(unionsToMerge, existingUnion.entityID, getCASTTime());
							
							if (newMergedUnion instanceof RelationUnion) {
								specifyUnionSourceAndTarget((RelationUnion)newMergedUnion, existingUnionConfig);
							}
							
							alreadyMergedUnions.put(existingUnion, newMergedUnion);
						} 
						// or simply fetch the already computed union
						else {
							newMergedUnion = alreadyMergedUnions.get(existingUnion);
						}
						// create and add a new union configuration with the new merged union
						UnionConfiguration newConfigWithMergedUnion = 
							createNewUnionConfiguration (existingUnionConfig, newMergedUnion, existingUnion);
						newUnionConfigs.add(newConfigWithMergedUnion);

					}				

				}
			}

			log("Total number of union configurations generated (before filtering): " + newUnionConfigs.size());

			newUnionConfigs = recompute(newUnionConfigs);

			// Add everything to the working memory
			AlternativeUnionConfigurations alters = buildNewAlternativeUnionConfigurations(newUnionConfigs);
			updateWM(alters); 
			
			// update the list of possible unions
			currentUnionConfigurations = newUnionConfigs;


		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	

	/**
	 * Restart the binding process from the start, by removing all existing unions
	 * and reconstructing the unions one by one
	 * 
	 * TODO: verify if the full rebinding method still works
	 */

	private void fullRebinding() {
		try {
			log("Perform full rebinding...");

			initializeUnionConfigurations();

			CASTData<Proxy>[] proxies = getWorkingMemoryEntries(Binder.BINDER_SA, Proxy.class);
			for (int i = 0 ; i < proxies.length; i++) {
				incrementalBinding(proxies[i].getData());
			}

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 

	
	private Vector<UnionConfiguration> recompute (Vector<UnionConfiguration> configs) {
		
		// Compute and normalise confidence scores for the union configurations
		ConfigurationFilter.computeConfidenceScoresForUnionConfigurations(configs);
					
		// based on these scores, compute the existence probabilities for each union
		BinderUtils.addProbExistsToUnions(configs);
		
		// Get the nbest configurations (with N as a parameter)
		if (nbestsFilter > 0) {
			configs = 
				ConfigurationFilter.getNBestUnionConfigurations (configs, nbestsFilter);
		}
		
		// Normalise the union distributions
		if (normaliseDistributions) {
			log("Normalisation of the probability distributions");
			normaliseDistributions(configs);
		}
		
		// Compute the marginal probability values for the individual feature values
		computeMarginalProbabilityValues(configs);
		
		return configs;
	}
	
	
	/**
	 * Normalise the union distribution in every configurations
	 * 
	 */
	private void normaliseDistributions (Vector<UnionConfiguration> configs) {
		
		// Loop on the union configurations
		for (Enumeration<UnionConfiguration> e = configs.elements() ; e.hasMoreElements(); ) {

			UnionConfiguration config = e.nextElement();				
 
			// Loop on the included unions
			for (int i = 0 ; i < config.includedUnions.length; i++) {
				
				// Normalise
				DistributionGeneration.normaliseDistribution(config.includedUnions[i].distribution, 1.0f);
			}
		}
	}
	
	/**
	 * Compute the marginal probability values for each feature value contained in the unions
	 * of the union configurations, and inserts this information in the "independentProb" 
	 * property of the feature value
	 * 
	 * @param configs the union configurations
	 */
	private void computeMarginalProbabilityValues(Vector<UnionConfiguration> configs) {

		// Loop on the union configurations
		for (Enumeration<UnionConfiguration> e = configs.elements() ; e.hasMoreElements(); ) {
			
			UnionConfiguration config = e.nextElement();	
			
			// Loop on the unions
			for (int i = 0 ; i < config.includedUnions.length ; i++) {

				Union union = config.includedUnions[i];
				
				// Loop on the features
				for (int j = 0; j < config.includedUnions[i].features.length ; j++){
					
					// Loop on the feature values
					for (int k =0; k < union.features[j].alternativeValues.length ; k++) {
						FeatureValuePair pair = new FeatureValuePair();
						pair.featlabel = union.features[j].featlabel;

						pair.featvalue = union.features[j].alternativeValues[k];
						union.features[j].alternativeValues[k].independentProb = 
							ProbabilityUtils.getMarginalProbabilityValue(union.distribution, pair); 
					}
				} 
			}
		}
	}
	
	
	private boolean isConfigurationAlreadyIncluded 
		(Vector<UnionConfiguration> configs, UnionConfiguration config) {
		
		for (int i = 0 ; i < configs.size(); i++) {
			UnionConfiguration curConfig = configs.elementAt(i);
			
			boolean foundMatchingConfig = true;
			
			if ((!curConfig.equals(config)) && (curConfig.includedUnions.length == config.includedUnions.length)) {
						
				HashMap<String, Union> entityIDsForCurConfig = new HashMap<String, Union>();
				for (int j = 0 ; j < curConfig.includedUnions.length ; j++) {
					entityIDsForCurConfig.put(curConfig.includedUnions[j].entityID, curConfig.includedUnions[j]);
				}
				
				for (int j = 0 ; j < config.includedUnions.length ; j++) {
					if (entityIDsForCurConfig.containsKey(config.includedUnions[j].entityID)) {
						Union unionInCurConfig = entityIDsForCurConfig.get(config.includedUnions[j].entityID);
						if (unionInCurConfig.features.length != config.includedUnions[j].features.length) {
							foundMatchingConfig = false;
						}
					}
					else {
						foundMatchingConfig = false;
					}
				}
			}
			else {
				foundMatchingConfig = false;
			}
			
			if (foundMatchingConfig) {
				return true;
			}
		}
		
		return false;
	}
	
	/**
	 * Get a vector containing all elements in the "proxies" array apart from proxyToExclude
	 * 
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


	
	private RelationUnion specifyUnionSourceAndTarget (RelationUnion union, UnionConfiguration config) {
		
		HashMap<String, String> unionForProxy = new HashMap<String, String>();
		for (int j = 0; j < config.includedUnions.length ; j++) {
			Union curUnion = config.includedUnions[j];
			for (int k = 0 ; k < curUnion.includedProxies.length; k++) {
				unionForProxy.put(curUnion.includedProxies[k].entityID, curUnion.entityID);
			}
		}
		
		union.usource = new Feature();
		union.usource.featlabel = "source";
		union.usource.alternativeValues = new AddressValue[union.psource.alternativeValues.length];
		
		for (int i = 0 ; i < union.psource.alternativeValues.length; i++) {
			String sourceId = ((AddressValue)union.psource.alternativeValues[i]).val;
			
			if (unionForProxy.containsKey(sourceId)) {
				AddressValue newSource = new AddressValue();
				newSource.independentProb = union.psource.alternativeValues[i].independentProb;
				newSource.timeStamp = union.psource.alternativeValues[i].timeStamp;
				newSource.val = unionForProxy.get(sourceId);
				union.usource.alternativeValues[i] = newSource;
			}
			else {
				errlog("WARNING: no union has been created for the proxy " + sourceId +
							", referenced by the relation union " + union.entityID);
			}
		}
		
		union.utarget = new Feature();
		union.utarget.featlabel = "target";
		union.utarget.alternativeValues = new AddressValue[union.ptarget.alternativeValues.length];

		for (int i = 0 ; i < union.ptarget.alternativeValues.length; i++) {
			String targetId = ((AddressValue)union.ptarget.alternativeValues[i]).val;
			
			if (unionForProxy.containsKey(targetId)) {
				AddressValue newTarget = new AddressValue();
				newTarget.independentProb = union.ptarget.alternativeValues[i].independentProb;
				newTarget.timeStamp = union.ptarget.alternativeValues[i].timeStamp;
				newTarget.val = unionForProxy.get(targetId);
				union.utarget.alternativeValues[i] = newTarget;
			}
			else {
				errlog("WARNING: no union has been created for the proxy " + targetId +
							", referenced by the relation union " + union.entityID);
			}
		}
		
		return union;
	}
	
	/**
	 * Check whether the union has been already computed
	 * 
	 * TODO: check whether this is still needed?
	 * @param union
	 * @param alreadyMergedUnions
	 * @return true if already computed, false otherwise
	 */
	public static boolean alreadyComputed(Union union, HashMap<Union, Union> alreadyMergedUnions) {
		for (Iterator<Union> i = alreadyMergedUnions.keySet().iterator() ; i.hasNext() ; ) {
			Union u = i.next();
			if (u.equals(union)  && u.entityID.equals(union.entityID) && u.timeStamp == union.timeStamp) {
				return true;
			}
		}
		return false;
	}

	
	/**
	 * Remove the union from the configuration
	 * 
	 * @param existingconfig
	 * @param unionToDelete
	 * @return
	 */
	private UnionConfiguration removeUnionFromConfig
	(UnionConfiguration existingconfig, Union unionToDelete) {

		// Aggregate the set of unions which need to be kept
		Vector<Union> unionsInConfig = new Vector<Union>();
		for (int i = 0; i < existingconfig.includedUnions.length ; i++) {
			Union curUnion = existingconfig.includedUnions[i];
			if (!curUnion.equals(unionToDelete)) {
				unionsInConfig.add(curUnion);
			}
		}
		
		// Update the included unions in the configuration
		existingconfig.includedUnions = new Union[unionsInConfig.size()];
		existingconfig.includedUnions = unionsInConfig.toArray(existingconfig.includedUnions);

		return existingconfig;
	}
	
	
	/**
	 * Build an AlternativeUnionConfigurations containing all configurations listed in
	 * configuration vector
	 * 
	 * @return the new AlternativeUnionConfigurations object
	 */
	private AlternativeUnionConfigurations buildNewAlternativeUnionConfigurations
		(Vector<UnionConfiguration> configs ) {

		AlternativeUnionConfigurations alters = new AlternativeUnionConfigurations();
		alters.alterconfigs = new UnionConfiguration[configs.size()];
		for (int i = 0; i < alters.alterconfigs.length ; i++) {
			alters.alterconfigs[i] = configs.elementAt(i); 
		}
		return alters;
	}


	/**
	 * Create a new union configuration based on an existing one and a new union 
	 * to add
	 * 
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
	 * 
	 * @param existingUnionConfig
	 * @param unionToAdd
	 * @param unionToRemove
	 * @return the new UnionConfiguration
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
	 * 
	 * @param existingUnionConfig
	 * @param unionToAdd
	 * @param unionsToRemove
	 * @return the new UnionConfiguration
	 */
	private UnionConfiguration createNewUnionConfiguration
		(UnionConfiguration existingUnionConfig, 
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
	 * 
	 * @param union1
	 * @param union2
	 * @return
	 */
	private boolean hasConflicts(Union union1, Union union2) {	
		for (int i = 0 ; i < union1.includedProxies.length ; i++) {
			Proxy proxyi = union1.includedProxies[i];
			for (int j = 0 ; j < union2.includedProxies.length ; j++) {
				Proxy proxyj = union2.includedProxies[j];
				if (proxyi.origin.address.subarchitecture.equals(proxyj.origin.address.subarchitecture)) {
					return true;
				}
			}
		}	
		if (union1 instanceof RelationUnion && 
				! (union2 instanceof RelationUnion)) {
			return true;
		}
		else if (! (union1 instanceof RelationUnion) && 
				union2 instanceof RelationUnion) {
			return true;
		}

		else if (!(union1 instanceof RelationUnion) && 
				(union1.features.length == 0 || union2.features.length == 0)) {
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
				getWorkingMemoryEntries(Binder.BINDER_SA, AlternativeUnionConfigurations.class);

			if (alterconfigs.length == 0) {
				addToWorkingMemory(newDataID(), Binder.BINDER_SA, configs);
			}
			else {
				overwriteWorkingMemory(alterconfigs[0].getID(), Binder.BINDER_SA,  configs);
			}
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
