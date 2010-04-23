package binder.components;

import java.util.LinkedList;
import java.util.List;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
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
 
	
	String beliefUpdateToIgnore = "";
	
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptUnionBelief.class);
							
								MultiModalBelief mmBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), _wmc.address, newDataID());				
								updatePointers(mmBelief, MultiModalBelief.class);
								
								insertBeliefInWM(mmBelief);

								addOffspring(beliefData.getData(), mmBelief.id);	
								beliefUpdateToIgnore = beliefData.getID();
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

							if (!beliefData.getID().equals(beliefUpdateToIgnore)) {

							List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
							for (WorkingMemoryAddress child : offspring) {
								if (existsOnWorkingMemory(child)) {
									MultiModalBelief childBelief = getMemoryEntry(child, MultiModalBelief.class);
									childBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), _wmc.address, childBelief.id);
									updateBeliefOnWM(childBelief);
								}
							}
							}
							else {
								log("ignore update, simple addition of offspring");
								beliefUpdateToIgnore = "";
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
