/**
 * 
 */
package execution.components;

import cast.AlreadyExistsOnWMException;
import cast.architecture.ManagedComponent;
import execution.slice.Action;

/**
 * Superclass for execution managing component.
 * 
 * @author nah
 *
 */
public abstract class AbstractExecutionManager extends ManagedComponent {
	
	protected void triggerExecution(Action _action) throws AlreadyExistsOnWMException {
		println(_action.getClass());
		addToWorkingMemory(newDataID(), _action);
	}

}
