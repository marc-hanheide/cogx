package binder.components;

import java.util.LinkedList;
import java.util.List;
import java.util.Vector;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.MultiModalBeliefBuilder;
import beliefmodels.builders.PerceptUnionBuilder;
import beliefmodels.utils.FeatureContentUtils;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import binder.abstr.BeliefWriter;
import binder.abstr.FakeComponent;
import binder.arch.BindingWorkingMemory;

public class MultiModalEstimation_fake extends FakeComponent {
 
		
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptUnionBelief.class);
							
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
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptUnionBelief.class);

							List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
							for (WorkingMemoryAddress child : offspring) {
								if (existsOnWorkingMemory(child)) {
									log("belief " + child.id + " exists on WM, overwriting");
									MultiModalBelief newChildBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), _wmc.address, child.id);
								
									MultiModalBelief existingBelief = getMemoryEntry(new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA), MultiModalBelief.class);
									newChildBelief.content = mergeBeliefContent(existingBelief.content, newChildBelief.content);
		
									updatePointers(newChildBelief, MultiModalBelief.class);
									updateBeliefOnWM(newChildBelief);
								}
								else {
									log("belief " + child.id + " does not exist on WM, creating it");
									MultiModalBelief childBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), _wmc.address, child.id);
									updatePointers(childBelief, MultiModalBelief.class);
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
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptUnionBelief.class);

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
