package execution.util;

import execution.slice.Action;
import execution.slice.TriBool;

/**
 * The interface used by {@link LocalActionStateManager} to trigger execution by
 * a component. Note that this will be called by separate thread than either the
 * component's run or event received threads, so access to state should be
 * synchronised if there is potential for corruption.
 * 
 * @author nah
 * 
 */
public interface ActionExecutor<ActionClass extends Action> {

	public interface ExecutionCompletionCallback {
		void executionComplete(TriBool _success);
	}

	/**
	 * Check if the executor will execute given this action.
	 * 
	 * @param _action
	 * @return
	 */
	boolean accept(ActionClass _action);

	/**
	 * Informs controller whether action execution blocks (i.e. only executes in
	 * the calling thread) or not (i.e. triggers behaviour in other components).
	 * 
	 * @return
	 */
	boolean isBlockingAction();

	/**
	 * Execute the action in a blocking form. Implement either this or
	 * execute(ExecutionCompletionCallback) depending on your action.
	 * 
	 * @return
	 */
	TriBool execute();

	/**
	 * Execute the action in a non-blocking form. The callback should be used to
	 * report completion. Do not perform any serious processing in this method.
	 * Implement either this or execute() depending on your action.
	 * 
	 * @return
	 */
	void execute(ExecutionCompletionCallback _callback);

	/**
	 * Called to stop the action before a non-block action has completed.
	 * Blocking actions cannot currently be executed.
	 */
	void stopExecution();
	
	/**
	 * Get the action class supported by this executor.
	 * @return
	 */
	Class<ActionClass> getActionClass();

}
