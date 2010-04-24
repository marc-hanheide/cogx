package binder.components;

import java.util.LinkedList;
import java.util.List;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
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

	
	String beliefUpdateToIgnore = "";

	
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, TemporalUnionBelief.class);
							
							StableBelief stableBelief = StableBeliefBuilder.createnewStableBelief(beliefData.getData(), _wmc.address, newDataID());
							
							updatePointers(stableBelief, StableBelief.class);
							
							insertBeliefInWM(stableBelief);

							addOffspring(beliefData.getData(), stableBelief.id);	
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
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, TemporalUnionBelief.class);

							if (!beliefData.getID().equals(beliefUpdateToIgnore)) {

							List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
							log("number of offspring for : " + beliefData.getData().id + ": "+ offspring.size());

							for (WorkingMemoryAddress child : offspring) {
								if (existsOnWorkingMemory(child)) {
									StableBelief childBelief = getMemoryEntry(child, StableBelief.class);
									childBelief =StableBeliefBuilder.createnewStableBelief(beliefData.getData(), _wmc.address, childBelief.id);
									updatePointers(childBelief, StableBelief.class);
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
