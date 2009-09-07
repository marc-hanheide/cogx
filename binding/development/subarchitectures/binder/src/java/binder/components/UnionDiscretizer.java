package binder.components;

import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.featvalues.StringValue;
import binder.utils.GradientDescent;
import binder.utils.ProbDistribUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class UnionDiscretizer extends ManagedComponent {
	
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

	
	public UnionConfiguration extractBestUnionConfiguration (AlternativeUnionConfigurations alterconfigs) {
		
				log("Number of alternative union configurations: "  + alterconfigs.alterconfigs.length);
				// Compute the best union configuration out of the possible ones
				UnionConfiguration bestConfiguration = 
					GradientDescent.getBestUnionConfiguration(alterconfigs);

				log("Best union configuration successfully computed");
				log("Number of unions in selected configuration: " + bestConfiguration.includedUnions.length);
				
				Vector<Union> unions = new Vector<Union>();
				// In the chosen union configuration, loop on the included unions, and compute
				// for each of them the instance with the maximum probability
				for (int i = 0 ; i < bestConfiguration.includedUnions.length ; i++) {
					Union uniondist = bestConfiguration.includedUnions[i];
			//		Union maxUnion = GradientDescent.getUnionWithMaximumProbability(uniondist);
					
					for (int j = 0; j < uniondist.features.length ; j++){
						for (int k =0; k < uniondist.features[j].alternativeValues.length ; k++) {
							FeatureValuePair pair = new FeatureValuePair();
							pair.featlabel = uniondist.features[j].featlabel;
							pair.featvalue = uniondist.features[j].alternativeValues[k];
					//		log("currently computing marginal prob for (" + pair.featlabel + ", " + ((StringValue)pair.featvalue).val + ")");
							uniondist.features[j].alternativeValues[k].independentProb = 
								ProbDistribUtils.getMarginalProbabilityValue(uniondist.distribution,pair); // / union.probExists;
						}
					} 
					
					unions.add(uniondist);
				} 
				
			
				
				UnionConfiguration discretizedConfig = new UnionConfiguration();
				discretizedConfig.includedUnions = new Union[unions.size()];
				discretizedConfig.includedUnions = unions.toArray(discretizedConfig.includedUnions);
				discretizedConfig.configProb = bestConfiguration.configProb;
				
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
