/**
 * 
 */
package execution.components;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import execution.slice.Action;
import execution.slice.ActionStatus;
import execution.slice.LogMessage;
import execution.slice.PrintMessage;
import execution.slice.TriBool;

/**
 * Test class to trigger executions (so to speak)
 * 
 * @author nah
 * 
 */
public class TestExecutionManager extends AbstractExecutionManager {

	@Override
	protected void runComponent() {

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
					triggerExecution(action);
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

}
