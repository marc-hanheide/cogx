package binder.components;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.builders.StableBeliefBuilder;
import binder.abstr.BeliefWriter;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class TemporalSmoothing_fake extends BeliefWriter {

	
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, TemporalUnionBelief.class);
							
							StableBelief mmBelief = StableBeliefBuilder.createnewStableBelief(beliefData.getData(), _wmc.address, newDataID());
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
