package binder.components;

import java.util.LinkedList;
import java.util.List;
import java.util.Vector;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.MultiModalBeliefBuilder;
import beliefmodels.builders.StableBeliefBuilder;
import beliefmodels.builders.TemporalUnionBuilder;
import beliefmodels.utils.FeatureContentUtils;
import binder.abstr.BeliefWriter;
import binder.abstr.FakeComponent;
import binder.arch.BindingWorkingMemory;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class TemporalSmoothing_fake extends FakeComponent {


	
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, TemporalUnionBelief.class);
							
							addOffspring(beliefData.getData(), newDataID());	
							updateBeliefOnWM(beliefData.getData());
								
						}	
			
						 catch (Exception e) {
								e.printStackTrace();
							}
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, TemporalUnionBelief.class);

							List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
							log("number of offspring for : " + beliefData.getData().id + ": "+ offspring.size());

							for (WorkingMemoryAddress child : offspring) {
								if (existsOnWorkingMemory(child)) {
									log("belief " + child.id + " exists on WM, merging content");
									StableBelief newChildBelief =StableBeliefBuilder.createnewStableBelief(beliefData.getData(), _wmc.address, child.id);

									StableBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA), StableBelief.class);
									newChildBelief.content = mergeBeliefContent(newChildBelief.content, existingBelief.content);
									
									updatePointers(newChildBelief, StableBelief.class);
									updateBeliefOnWM(newChildBelief);
								}
								else {
									log("belief " + child.id + " does not exist on WM, creating it");
									StableBelief childBelief =StableBeliefBuilder.createnewStableBelief(beliefData.getData(), _wmc.address, child.id);
									updatePointers(childBelief, StableBelief.class);
									insertBeliefInWM(childBelief);
								}
							}
							
						}	

						catch (Exception e) {
							e.printStackTrace();
						} 
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, TemporalUnionBelief.class);

							List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;

							for (WorkingMemoryAddress child : offspring) {
								if (existsOnWorkingMemory(child)) {
									deleteBeliefOnWM(child.id);
								}
							}
							
						}	

						catch (Exception e) {
							e.printStackTrace();
						} 
					}
				}
		);
	}
	
	
}
