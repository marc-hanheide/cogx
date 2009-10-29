package binder.utils;

import java.util.HashMap;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.filtering.ConfigurationFilter;

public class UnionConfigurationUtils {


	
	/**
	 * Remove the union from the configuration
	 * 
	 * @param existingconfig
	 * @param unionToDelete
	 * @return
	 */
	public static  UnionConfiguration removeUnionFromConfig
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
	public static  AlternativeUnionConfigurations buildNewAlternativeUnionConfigurations
		(Vector<UnionConfiguration> configs ) {

		UnionConfiguration[] alterconfigs = new UnionConfiguration[configs.size()];
		for (int i = 0; i < alterconfigs.length ; i++) {
			alterconfigs[i] = configs.elementAt(i); 
		}
		AlternativeUnionConfigurations alters = new AlternativeUnionConfigurations(alterconfigs);

		return alters;
	}

	

	public static  UnionConfiguration createNewUnionConfigurationWithOrphanProxy
		(UnionConfiguration existingUnionConfig, Proxy orphan) {
	
		float configProb = -1.0f;
		Union[] includedUnions = existingUnionConfig.includedUnions;
		Proxy[] orphanProxies;
		
		if (existingUnionConfig.orphanProxies != null) {
		orphanProxies = new Proxy[existingUnionConfig.orphanProxies.length +1 ];
		for (int t = 0; t < existingUnionConfig.orphanProxies.length ; t++) {
			orphanProxies[t] = existingUnionConfig.orphanProxies[t];
		}
		orphanProxies[existingUnionConfig.orphanProxies.length] = orphan;
		}
		else {
			orphanProxies = new Proxy[1];
			orphanProxies[0] = orphan;
		}
		
		UnionConfiguration unionConfigWithOrphanProxy = new UnionConfiguration(includedUnions, orphanProxies, configProb);

		return unionConfigWithOrphanProxy;
	}


	/**
	 * Create a new union configuration based on an existing one and a new union 
	 * to add
	 * 
	 * @param existingUnionConfig
	 * @param unionToAdd
	 * @return
	 */
	public static  UnionConfiguration createNewUnionConfiguration
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
	public static  UnionConfiguration createNewUnionConfiguration
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
	public static  UnionConfiguration createNewUnionConfiguration
		(UnionConfiguration existingUnionConfig, 
			Union unionToAdd, Vector<Union> unionsToRemove) {

		float configProb = -1.0f;
		int nbUnions = existingUnionConfig.includedUnions.length + 1 - unionsToRemove.size();
		Union[] includedUnions = new Union[nbUnions];
		Proxy[] orphanProxies = existingUnionConfig.orphanProxies;

		int count = 0;
		for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {
			if (!unionsToRemove.contains(existingUnionConfig.includedUnions[i])) {
				includedUnions[i- count] = existingUnionConfig.includedUnions[i];
			}
			else {
				count ++;
			}
		}

		includedUnions[nbUnions - 1] = unionToAdd;
		UnionConfiguration newConfig = new UnionConfiguration(includedUnions, orphanProxies, configProb);

		return newConfig;
	}
	
	public static void addProbsOfConfig1IntoConfig2 (UnionConfiguration config1, UnionConfiguration config2) {
		
		for (int i = 0 ; i < config1.includedUnions.length ; i++) {
			Union union1 = config1.includedUnions[i];
			if (union1.probExists == 0.0f) {
				union1.probExists = ConfigurationFilter.getProbabilitiesSum (union1.distribution);
			}
			for (int j = 0 ; j < config2.includedUnions.length; j++) {
				Union union2 = config2.includedUnions[j];
				
				if (union1.entityID.equals(union2.entityID) && 
						union1.includedProxies.length == union2.includedProxies.length && 
						union1.features.length == union2.features.length) {
					
					union2.probExists = union2.probExists +  union1.probExists;
				}
					
			}
		}
	}
	public static UnionConfiguration getSimilarUnionConfig
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
				return curConfig;
			}
		}
		
		return null;
	}
	
}
