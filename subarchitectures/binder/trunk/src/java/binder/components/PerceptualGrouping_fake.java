package binder.components;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.PerceptUnionBuilder;
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

public class PerceptualGrouping_fake extends BeliefWriter {

	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptBelief.class);
							
								PerceptUnionBelief union = PerceptUnionBuilder.createNewSingleUnionBelief(beliefData.getData(), _wmc.address, newDataID());
								
								updatePointersInCurrentBelief(union);
								updatePointersInOtherBeliefs(union);
								
								insertBeliefInWM(union);

								addOffspringToPercept(beliefData.getData(), 
										new WorkingMemoryAddress(union.id, BindingWorkingMemory.BINDER_SA));								
						}	

						catch (Exception e) {
							e.printStackTrace();
						} 
					}
				}
		);
	}


	private void addOffspringToPercept (PerceptBelief percept, WorkingMemoryAddress addressUnion) {

		if (percept.hist != null && percept.hist instanceof CASTBeliefHistory) {
			((CASTBeliefHistory)percept.hist).offspring.add(addressUnion);
		}
	}


	private void updatePointersInCurrentBelief (Belief newBelief) {
		try {

			for (FeatureValueProbPair pointer : FeatureContentUtils.getAllPointerValuesInBelief(newBelief)) {

				WorkingMemoryAddress point = ((PointerValue)pointer.val).beliefId ;
				PerceptBelief belief = getMemoryEntry(point, PerceptBelief.class);
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

					CASTData<PerceptUnionBelief>[] unions = getWorkingMemoryEntries(PerceptUnionBelief.class);

					for (int i = 0 ; i < unions.length ; i++ ) {
						PerceptUnionBelief union = unions[i].getData();
						for (FeatureValueProbPair pointerValueInUnion : FeatureContentUtils.getAllPointerValuesInBelief(union)) {
							PointerValue val = (PointerValue)pointerValueInUnion.val;
							if (val.beliefId.equals(ancestor)) {
								((PointerValue)pointerValueInUnion.val).beliefId = 
									new WorkingMemoryAddress(newBelief.id, BindingWorkingMemory.BINDER_SA);
								updateBeliefOnWM(union);
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
