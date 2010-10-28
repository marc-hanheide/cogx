package execution.util;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;

import execution.slice.Action;



/**
 * Helper class to simplify writing a non-blocking subclass of
 * {@link ActionExecutor}. This contains a field to store the callback and
 * handles error cases.
 * 
 * @author nah
 * 
 */
public abstract class BlockingActionExecutor<ActionType extends Action> extends ComponentActionExecutor {

	private final Class<ActionType> m_actCls;
	private ActionType m_action;
	
	public BlockingActionExecutor(ManagedComponent _component, Class<ActionType> _exeCls) {
		super(_component);
		m_actCls = _exeCls;
	}
	
	
	protected ActionType getAction() {
		assert(m_action != null);
		return m_action;
	}
	
	@Override
	public boolean accept(Action _action) {
		if(_action.getClass().equals(m_actCls)) {
			m_action = m_actCls.cast(_action);
			return true;
		}
		else {
			Logger.getLogger(getClass()).info("Refusing to execute action of type: " + _action.getClass());
			return false;
		}
		

	}

	@Override
	public boolean isBlockingAction() {
		return true;
	}


	@Override
	public void execute(ExecutionCompletionCallback _callback) {
		throw new RuntimeException("Non-blocking action should not be called");
	
	}

	@Override
	public void stopExecution() {
		//can't stop blocking execution
	}
	
}
