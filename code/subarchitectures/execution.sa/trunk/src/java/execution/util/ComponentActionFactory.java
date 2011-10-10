package execution.util;

import java.lang.reflect.Constructor;

import cast.architecture.ManagedComponent;
import execution.slice.Action;

/**
 * Create factory that can create action executor which can be constructed using
 * a {@link ManagedComponent} as the sole argument. The {@link ActionExecutor} class specific MUST have a public constructor and the sole argument must be written as {@link ManagedComponent} not any subclass.
 * 
 * @author nah
 * 
 * @param <ExecutorType>
 */
public class ComponentActionFactory<ActionType extends Action, ExecutorType extends ActionExecutor<ActionType>>
		implements ActionExecutorFactory<ActionType> {

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
	public ActionExecutor<ActionType> getActionExecutor() {
		try {
			Constructor<ExecutorType> constructor = m_exeClass
					.getConstructor(ManagedComponent.class);
			return constructor.newInstance(m_component);
		} catch (Exception e) {
			for(Constructor<?> c : m_exeClass.getConstructors()) {
				m_component.println(c);
			}
			
			m_component.logException(e);
			throw new RuntimeException(e);
		}

	}

}
