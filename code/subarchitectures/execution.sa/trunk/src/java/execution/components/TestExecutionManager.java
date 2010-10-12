/**
 * 
 */
package execution.components;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import execution.slice.Action;
import execution.slice.ActionStatus;
import execution.slice.TriBool;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LogMessage;
import execution.slice.actions.PrintMessage;
import execution.util.ActionMonitor;

/**
 * Test class to trigger executions (so to speak)
 * 
 * @author nah
 * 
 */
public class TestExecutionManager extends AbstractExecutionManager {

	
	private class DummyCallback implements ActionMonitor {

		public void actionComplete(Action _action) {
			println("Action complete: " + _action.getClass());
		}
		
	}
	
	@Override
	protected void runComponent() {

		testSpatial();
//	simpleTest();
	}

	/**
	 * 
	 */
	private void testSpatial() {
		//time to create a place
		sleepComponent(20000);
		
		try {
			println("triggering");
			triggerExecution(new GoToPlace(ActionStatus.PENDING, TriBool.TRIINDETERMINATE, 2), new DummyCallback());
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}
	}

	/**
	 * 
	 */
	private void simpleTest() {
		while (isRunning()) {
			// Randomly selects a type of action to be performed then triggers
			// it. 

			Action action = null;

			if (Math.random() > 0.5) {
				action = new LogMessage(ActionStatus.PENDING,
						TriBool.TRIINDETERMINATE, "Goodbye world!");
			} else {
				action = new PrintMessage(ActionStatus.PENDING,
						TriBool.TRIINDETERMINATE, "Hello world (again)!");
			}

			try {
				try {
					triggerExecution(action, new DummyCallback());
				} catch (DoesNotExistOnWMException e) {
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					e.printStackTrace();
				}
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
			}

			sleepComponent((long) (Math.random() * 2000));

		}
	}

	@Override
	public boolean isPaused() {
		return false;
	}

}
