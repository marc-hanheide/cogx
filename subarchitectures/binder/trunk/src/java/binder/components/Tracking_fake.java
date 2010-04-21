package binder.components;

import java.util.List;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.MultiModalBeliefBuilder;
import beliefmodels.builders.TemporalUnionBuilder;
import beliefmodels.utils.FeatureContentUtils;
import binder.abstr.BeliefWriter;
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

public class Tracking_fake extends BeliefWriter {
 
	    
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						 
						try {
							CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(_wmc.address, MultiModalBelief.class);
							
							TemporalUnionBelief tunion = TemporalUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), _wmc.address, newDataID());							

							updatePointersInCurrentBelief(tunion);
							updatePointersInOtherBeliefs(tunion);
								
							addOffspringToMMBelief(beliefData.getData(), 
									new WorkingMemoryAddress(tunion.id, BindingWorkingMemory.BINDER_SA));	

							insertBeliefInWM(tunion);

						}	
			
						 catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							}
						 catch (UnknownSubarchitectureException e) {	
							e.printStackTrace();
						} 
						 catch (BeliefException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							} catch (AlreadyExistsOnWMException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				}
		);
		
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(_wmc.address, MultiModalBelief.class);

							List<WorkingMemoryAddress> offspring = ((CASTBeliefHistory)beliefData.getData().hist).offspring;
							
							for (WorkingMemoryAddress child : offspring) {
								if (existsOnWorkingMemory(child)) {
									TemporalUnionBelief childBelief = getMemoryEntry(child, TemporalUnionBelief.class);
									childBelief =TemporalUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), _wmc.address, childBelief.id);
									updateBeliefOnWM(childBelief);
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
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(_wmc.address, MultiModalBelief.class);

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
	
	

	private void addOffspringToMMBelief (MultiModalBelief mmBelief, WorkingMemoryAddress addressTUnionBelief) {

		if (mmBelief.hist != null && mmBelief.hist instanceof CASTBeliefHistory) {
			((CASTBeliefHistory)mmBelief.hist).offspring.add(addressTUnionBelief);
		}
	}


	private void updatePointersInCurrentBelief (Belief newBelief) {
		try {

			for (FeatureValueProbPair pointer : FeatureContentUtils.getAllPointerValuesInBelief(newBelief)) {

				WorkingMemoryAddress point = ((PointerValue)pointer.val).beliefId ;
				if (existsOnWorkingMemory(point)) {
					Belief belief = getMemoryEntry(point, Belief.class);
				if (((CASTBeliefHistory)belief.hist).offspring.size() > 0) {
					((PointerValue)pointer.val).beliefId = ((CASTBeliefHistory)belief.hist).offspring.get(0);	
				}
				}
				else {
					log("warning: belief " + point.id + " does not exist yet on WM");
				}
			}
				
		}
		catch (Exception e) {
			e.printStackTrace();
		}

	}


	private void updatePointersInOtherBeliefs (Belief newBelief) {
		try {

			if (newBelief.hist != null && newBelief.hist instanceof CASTBeliefHistory) {

				for (WorkingMemoryAddress ancestor: ((CASTBeliefHistory)newBelief.hist).ancestors) {

					CASTData<TemporalUnionBelief>[] tunions = getWorkingMemoryEntries(TemporalUnionBelief.class);

					for (int i = 0 ; i < tunions.length ; i++ ) {
						TemporalUnionBelief tunion = tunions[i].getData();
						for (FeatureValueProbPair pointerValueInUnion : FeatureContentUtils.getAllPointerValuesInBelief(tunion)) {
							PointerValue val = (PointerValue)pointerValueInUnion.val;
							if (val.beliefId.equals(ancestor)) {
								((PointerValue)pointerValueInUnion.val).beliefId = 
									new WorkingMemoryAddress(newBelief.id, BindingWorkingMemory.BINDER_SA);
								updateBeliefOnWM(tunion);
							}
						}
					}
				} 
			}


		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
