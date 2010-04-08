package binder.components;

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


/**
 * Simple class to demonstrate how to monitor the working memory for new changes 
 * 
 * @author plison
 *
 */
public class BinderMonitor extends ManagedComponent {


	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						newPerceptBeliefAdded(_wmc);
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						perceptBeliefUpdated(_wmc);
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						perceptBeliefDeleted(_wmc);
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						unionBeliefAdded(_wmc);
					}
				}
		);
	}
	
	
	protected void unionBeliefAdded(WorkingMemoryChange _wmc) {
		try {
			CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					PerceptUnionBelief.class);
			PerceptUnionBelief newBelief = beliefData.getData();
			
			// here you can do what you want with the belief
			
		}
		 catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			}
		 catch (UnknownSubarchitectureException e) {	
			e.printStackTrace();
		} 
		
	}


	protected void perceptBeliefDeleted(WorkingMemoryChange _wmc) {
		// TODO Auto-generated method stub
		
	}


	protected void perceptBeliefUpdated(WorkingMemoryChange _wmc) {
		// TODO Auto-generated method stub
		
	}


	private void newPerceptBeliefAdded(WorkingMemoryChange _wmc) {
		// TODO Auto-generated method stub
	}
}
