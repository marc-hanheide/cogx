package binder.components;

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
import binder.arch.BindingWorkingMemory;

public class MultiModalEstimation_fake extends BeliefWriter {
 
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptUnionBelief.class);
							
								MultiModalBelief mmBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), _wmc.address, newDataID());				
								
								updatePointersInCurrentBelief(mmBelief);
								updatePointersInOtherBeliefs(mmBelief);
								
								insertBeliefInWM(mmBelief);

								addOffspringToUnion(beliefData.getData(), 
										new WorkingMemoryAddress(mmBelief.id, BindingWorkingMemory.BINDER_SA));	
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
	}
	

	private void addOffspringToUnion (PerceptUnionBelief union, WorkingMemoryAddress addressMMBelief) {

		if (union.hist != null && union.hist instanceof CASTBeliefHistory) {
			((CASTBeliefHistory)union.hist).offspring.add(addressMMBelief);
		}
	}


	private void updatePointersInCurrentBelief (Belief newBelief) {
		try {

			for (FeatureValueProbPair pointer : FeatureContentUtils.getAllPointerValuesInBelief(newBelief)) {

				WorkingMemoryAddress point = ((PointerValue)pointer.val).beliefId ;
				PerceptUnionBelief belief = getMemoryEntry(point, PerceptUnionBelief.class);
				if (((CASTBeliefHistory)belief.hist).offspring.size() > 0) {
					((PointerValue)pointer.val).beliefId = ((CASTBeliefHistory)belief.hist).offspring.get(0);	
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

					CASTData<MultiModalBelief>[] mmbeliefs = getWorkingMemoryEntries(MultiModalBelief.class);

					for (int i = 0 ; i < mmbeliefs.length ; i++ ) {
						MultiModalBelief mmbelief = mmbeliefs[i].getData();
						for (FeatureValueProbPair pointerValueInUnion : FeatureContentUtils.getAllPointerValuesInBelief(mmbelief)) {
							PointerValue val = (PointerValue)pointerValueInUnion.val;
							if (val.beliefId.equals(ancestor)) {
								((PointerValue)pointerValueInUnion.val).beliefId = 
									new WorkingMemoryAddress(newBelief.id, BindingWorkingMemory.BINDER_SA);
								updateBeliefOnWM(mmbelief);
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
