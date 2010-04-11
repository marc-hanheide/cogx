package binder.components;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.builders.TemporalUnionBuilder;
import binder.abstr.BeliefWriter;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
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
							
							TemporalUnionBelief mmBelief = TemporalUnionBuilder.createNewSingleTemporalUnionBuilder(beliefData.getData(), newDataID());
								insertBeliefInWM(mmBelief);
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
}
