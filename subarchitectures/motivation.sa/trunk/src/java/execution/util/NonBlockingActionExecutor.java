package execution.util;

import execution.slice.TriBool;

/**
 * Helper class to simplify writing a non-blocking subclass of
 * {@link ActionExecutor}. This contains a field to store the callback and
 * handles error cases.
 * 
 * @author nah
 * 
 */
public abstract class NonBlockingActionExecutor implements ActionExecutor {

	private ExecutionCompletionCallback m_callback;

	@Override
	public TriBool execute() {
		throw new RuntimeException("Blocking action should not be called");
	}

	@Override
	public boolean isBlockingAction() {
		return false;
	}

	@Override
	public void execute(ExecutionCompletionCallback _callback) {
		m_callback = _callback;
		executeAction();
	}

	public abstract void executeAction();
	
	/**
	 * Should be called when execution has completed. Result is passed back to
	 * callback.
	 * 
	 * @param _success
	 */
	protected void executionComplete(TriBool _success) {
		m_callback.executionComplete(_success);
	}

}
