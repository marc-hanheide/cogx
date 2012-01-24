package execution.util;


import cast.architecture.ManagedComponent;

/**
 * Creates executors that do nothing.
 */
public class DoNothingActionExecutorFactory implements ActionExecutorFactory {

	private final ManagedComponent m_component;

	public DoNothingActionExecutorFactory(ManagedComponent _component) {
		m_component = _component;
	}

	@Override
	public ActionExecutor getActionExecutor() {
		return new DoNothingExecutor(m_component);
	}

}