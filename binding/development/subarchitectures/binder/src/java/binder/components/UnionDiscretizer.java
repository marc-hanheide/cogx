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
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.FeatureValuePair;
import binder.utils.GradientDescent;
import binder.utils.ProbabilityUtils;
import binder.utils.UnionConstructor;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class UnionDiscretizer extends ManagedComponent {

	public boolean onlyMaxFeatureValues = true;
	
	@Override
	public void start() {

		// if the set of possible union configurations has been updated, update the
		// monitor accordingly
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AlternativeUnionConfigurations.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					AlternativeUnionConfigurations alterconfigs = 
						getMemoryEntry(_wmc.address, AlternativeUnionConfigurations.class);

					UnionConfiguration uc = extractBestUnionConfiguration(alterconfigs);

					addBestUnionConfigurationToWM(uc);
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

	

	public UnionConfiguration extractBestUnionConfiguration 
	(AlternativeUnionConfigurations alterconfigs) {

		log("--------START DISCRETISATION ----------");
		long initTime = System.currentTimeMillis();


		log("Number of alternative union configurations: "  + alterconfigs.alterconfigs.length);
		// Compute the best union configuration out of the possible ones
		UnionConfiguration bestConfiguration = 
			GradientDescent.getBestUnionConfiguration(alterconfigs);

		log("Best union configuration successfully computed");
		log("Number of unions in selected configuration: " + 
				bestConfiguration.includedUnions.length);

		Vector<Union> unions = new Vector<Union>();
		// In the chosen union configuration, loop on the included unions, and compute
		// for each of them the instance with the maximum probability
		for (int i = 0 ; i < bestConfiguration.includedUnions.length ; i++) {
			Union union = bestConfiguration.includedUnions[i];
			
			if (onlyMaxFeatureValues) {
				union = GradientDescent.getUnionWithMaximumProbability(union);
			}

			for (int j = 0; j < union.features.length ; j++){
				for (int k =0; k < union.features[j].alternativeValues.length ; k++) {
					FeatureValuePair pair = new FeatureValuePair();
					pair.featlabel = union.features[j].featlabel;

					pair.featvalue = union.features[j].alternativeValues[k];
					//		log("currently computing marginal prob for (" + 
					// 		pair.featlabel + ", " + BinderUtils.toString(pair.featvalue) + ")");
					union.features[j].alternativeValues[k].independentProb = 
						ProbabilityUtils.getMarginalProbabilityValue(union.distribution,pair); // / union.probExists;
				}
			} 

			unions.add(union);
		} 

		UnionConfiguration discretizedConfig = new UnionConfiguration();
		discretizedConfig.includedUnions = new Union[unions.size()];
		discretizedConfig.includedUnions = unions.toArray(discretizedConfig.includedUnions);
		discretizedConfig.configProb = bestConfiguration.configProb;

		long finalTime = System.currentTimeMillis();
		log("Total discretisation time: " + (finalTime - initTime)/1000.0 + " seconds");
		log("--------STOP DISCRETISATION ----------");

		return discretizedConfig;
	}

	protected void addBestUnionConfigurationToWM(UnionConfiguration config) {

		try {
			CASTData<UnionConfiguration>[] existingconfigs = 
				getWorkingMemoryEntries(UnionConfiguration.class);
			if (existingconfigs != null && existingconfigs.length > 0) {
				overwriteWorkingMemory (existingconfigs[0].getID(), config);
			}
			else {				
				addToWorkingMemory(newDataID(), config);
			}
			log("Union configuration succesfully added/updated");

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

}
