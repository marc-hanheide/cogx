package execution.util;

import cast.architecture.ManagedComponent;
import execution.slice.Action;
import execution.slice.TriBool;

/**
 * Helper class to simplify writing a non-blocking subclass of
 * {@link ActionExecutor}. This contains a field to store the callback and
 * handles error cases.
 * 
 * @author nah
 * 
 */
public abstract class NonBlockingActionExecutor<ActionType extends Action>
		extends ComponentActionExecutor {

	private ExecutionCompletionCallback m_callback;
	private final Class<ActionType> m_actCls;
	private Action m_action;
	
	
	public NonBlockingActionExecutor(ManagedComponent _component, Class<ActionType> _actCls) {
		super(_component);
		m_actCls = _actCls;
	}

	protected abstract boolean acceptAction(ActionType _action);

	@Override
	public boolean accept(Action _action) {
		m_action = _action;
		return acceptAction(getAction());
	}
	
	protected ActionType getAction() {
		return m_actCls.cast(m_action);
	}

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
