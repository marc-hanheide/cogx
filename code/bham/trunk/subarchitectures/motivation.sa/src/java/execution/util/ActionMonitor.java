/**
 * 
 */
package execution.util;

import com.sun.tools.example.debug.bdi.ExecutionManager;

import execution.slice.Action;

/**
 * Interface used by the {@link ExecutionManager} to monitor actions which it has triggered.
 * 
 * @author nah
 *
 */
public interface ActionMonitor {

	/**
	 * Called when the triggered action is complete.
	 * @param _action
	 */
	void actionComplete(Action _action);
}
