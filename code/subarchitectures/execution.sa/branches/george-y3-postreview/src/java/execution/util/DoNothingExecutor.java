package execution.util;

import cast.architecture.ManagedComponent;
import execution.slice.Action;
import execution.slice.TriBool;

/**
 * Dummy executor that sleeps for 3 seconds before reporting success.
 * @author nah
 *
 */
public class DoNothingExecutor extends NonBlockingCompleteOnOperationExecutor<Action> {

	private class CompleteAction extends SleepyThread {
		public CompleteAction(long _sleepyTime) {
			super(_sleepyTime);
		}
		@Override
		protected void doSomething() {		
			executionComplete(TriBool.TRITRUE);
		}
		
	}
	
	public DoNothingExecutor(ManagedComponent _component) {
		super(_component, Action.class);
	}


	@Override
	public void executeAction() {
		getComponent().println("Doing nothing for 3 seconds. I hope that's OK with you.");
		new CompleteAction(3000).start();
	}


	@Override
	protected boolean acceptAction(Action _action) {
		return true;
	}

	
}