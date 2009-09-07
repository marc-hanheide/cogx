/**
 * 
 */
package execution.components;

import cast.AlreadyExistsOnWMException;
import autogen.Planner.Completion;
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
		PrintMessage printAction = new PrintMessage(Completion.PENDING,
				TriBool.TRIINDETERMINATE, "Hello world (again)!");
		try {
			triggerExecution(printAction);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

}
