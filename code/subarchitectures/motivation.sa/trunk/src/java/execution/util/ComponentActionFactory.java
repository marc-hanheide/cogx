package execution.util;

import java.lang.reflect.Constructor;

import cast.architecture.ManagedComponent;

/**
 * Create factory that can create action executor which can be constructed using
 * a {@link ManagedComponent} as the sole argument.
 * 
 * @author nah
 * 
 * @param <ExecutorType>
 */
public class ComponentActionFactory<ExecutorType extends ActionExecutor>
		implements ActionExecutorFactory {

	private final ManagedComponent m_component;
	private Class<ExecutorType> m_exeClass;

	public ComponentActionFactory(ManagedComponent _component,
			Class<ExecutorType> _exeClass) {
		m_component = _component;
		m_exeClass = _exeClass;
	}

	protected ManagedComponent getComponent() {
		return m_component;
	}

	@Override
	public ActionExecutor getActionExecutor() {
		try {
			Constructor<ExecutorType> constructor = m_exeClass
					.getConstructor(ManagedComponent.class);
			return constructor.newInstance(m_component);

		} catch (Exception e) {
			m_component.logException(e);
			throw new RuntimeException(e);
		}

	}

}
