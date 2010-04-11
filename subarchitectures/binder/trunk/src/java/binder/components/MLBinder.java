package binder.components;

import beliefmodels.autogen.beliefs.PerceptBelief;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class MLBinder<Type extends Object> extends ManagedComponent {
	
	public boolean LOGGING = true;
	
	private static Object belief_type = new PerceptBelief();
	
	@Override
	public void start() {
		log("Initialize Binder... ");
		
		// change filter for monitoring of Belief insertions
		this.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(PerceptBelief.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange wmc) throws CASTException {
				actionBeliefAdded(wmc);
			}
		});
		
		// change filter for monitoring of Belief modifications
		this.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(PerceptBelief.class,
				WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange wmc) throws CASTException {
				actionBeliefModified(wmc);
			}
		});
		
		// change filter for monitoring of Belief deletion
		this.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(PerceptBelief.class,
				WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange wmc) throws CASTException {
				actionBeliefDeleted(wmc);
			}
		});
	}
	
	protected void actionBeliefDeleted(WorkingMemoryChange wmc) {
		// TODO Auto-generated method stub
		
	}

	protected void actionBeliefModified(WorkingMemoryChange wmc) {
		// TODO Auto-generated method stub
		
	}

	protected void actionBeliefAdded(WorkingMemoryChange wmc) {
		// TODO Auto-generated method stub
		
	}

	private void log(String s) {
		if (LOGGING) {
			System.out.println("[PerceptualGroupingBinder] " + s);
		}
	}
}
