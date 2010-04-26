package binder.components;

import java.util.LinkedList;
import java.util.List;
import java.util.Vector;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.PerceptUnionBuilder;
import beliefmodels.utils.FeatureContentUtils;
import binder.abstr.BeliefWriter;
import binder.abstr.FakeComponent;
import binder.arch.BindingWorkingMemory;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class PerceptualGrouping_fake extends FakeComponent {

		
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptBelief.class);

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
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptBelief.class);

							log("overwriting belief " + beliefData.getID());

							List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;

							log("number of offspring for belief " + beliefData.getID() + ": " + offspring.size());

							for (WorkingMemoryAddress child : offspring) {
								if (existsOnWorkingMemory(child)) {
									log("belief " + child.id + " exists on WM, overwriting");
									PerceptUnionBelief newChildBelief = PerceptUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), _wmc.address, child.id);
								
									PerceptUnionBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA), PerceptUnionBelief.class);
									newChildBelief.content = mergeBeliefContent(existingBelief.content, newChildBelief.content);
	
									updatePointers(newChildBelief, PerceptUnionBelief.class);
									updateBeliefOnWM(newChildBelief);
								}
								else {
									log("belief " + child.id + " does not exist on WM, creating it");
									PerceptUnionBelief childBelief = PerceptUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), _wmc.address, child.id);
									
									updatePointers(childBelief, PerceptUnionBelief.class);
									insertBeliefInWM(childBelief);
								}
							}
							
							if (offspring.size() == 0) {
								log("WARNING: " + beliefData.getID() + " has no offspring, something wrong here");
							}
						}	

						catch (Exception e) {
							e.printStackTrace();
						} 
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptBelief.class);

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
