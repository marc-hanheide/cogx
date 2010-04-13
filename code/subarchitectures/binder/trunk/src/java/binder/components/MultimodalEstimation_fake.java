package binder.components;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.builders.MultiModalBeliefBuilder;
import beliefmodels.builders.PerceptUnionBuilder;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import binder.abstr.BeliefWriter;

public class MultimodalEstimation_fake extends BeliefWriter {
 
	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptUnionBelief.class);
							
								MultiModalBelief mmBelief = MultiModalBeliefBuilder.createNewMultiModalBelief(beliefData.getData(), _wmc.address, newDataID());
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
