package execution.util;

import cast.architecture.ManagedComponent;

public abstract class ComponentActionExecutor implements ActionExecutor {

	private final ManagedComponent m_component;
	
	public ComponentActionExecutor(ManagedComponent _component) {
		m_component = _component;
	}

	protected ManagedComponent getComponent() {
		return m_component;
	}
	
}
