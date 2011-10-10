package execution.util;

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
public abstract class BlockingActionExecutor<ActionType extends Action> extends
		ComponentActionExecutor<ActionType> {

	private final Class<ActionType> m_actCls;
	private ActionType m_action;

	public BlockingActionExecutor(ManagedComponent _component,
			Class<ActionType> _exeCls) {
		super(_component);
		m_actCls = _exeCls;
		
//		getComponent().println("constructued with action class: " + m_actCls);
//		System.out.println("constructued with action class: " + m_actCls);
	}

	protected ActionType getAction() {
		assert (m_action != null);
		return m_action;
	}

	@Override
	public boolean accept(Action _action) {
		if (_action.getClass().equals(m_actCls)) {
			m_action = m_actCls.cast(_action);
			log("accepted action: " + m_actCls);
			return true;
		} else {
			getComponent().getLogger()
					.info("Refusing to execute action of type: "
							+ _action.getClass() + " instead of configured type " + m_actCls);
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
		// can't stop blocking execution
	}

	@Override
	public Class<ActionType> getActionClass() {
		return m_actCls;
	}
	
}
