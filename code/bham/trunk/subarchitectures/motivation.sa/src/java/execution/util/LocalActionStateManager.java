package execution.util;

import java.util.HashMap;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import execution.slice.Action;
import execution.slice.ActionExecutionException;
import execution.slice.PrintMessage;

/**
 * 
 * Manages interactions with WM for action execution.
 * 
 * @author nah
 * 
 */
public class LocalActionStateManager {

	/**
	 * The component which will do the execution.
	 */
	private final ManagedComponent m_component;

	/**
	 * Map of classes to factories used to generate the execution objects.
	 */
	private HashMap<String, ActionExecutorFactory> m_executorFactories;

	public LocalActionStateManager(ManagedComponent _component) {
		m_component = _component;
	}

	public <Type extends Action> void registerActionType(
			Class<Type> _actionCls, ActionExecutorFactory _executionFactory) {

		// lazy creation
		if (m_executorFactories == null) {
			m_executorFactories = new HashMap<String, ActionExecutorFactory>();
		}

		m_executorFactories.put(CASTUtils.typeName(_actionCls), _executionFactory);

		m_component.addChangeFilter(ChangeFilterFactory
				.createLocalTypeFilter(_actionCls),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							newActionReceived(_wmc);
						} catch (ActionExecutionException e) {
							System.out.println(e.message);
							e.printStackTrace();
						} catch (DoesNotExistOnWMException e) {
							System.out.println(e.message);
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							System.out.println(e.message);
							e.printStackTrace();
						}
					}
				});
		
		System.out.println("registered exection for: " + _actionCls);
	}

	protected void newActionReceived(WorkingMemoryChange _wmc) throws ActionExecutionException, DoesNotExistOnWMException, UnknownSubarchitectureException {
		assert(m_executorFactories != null);
		ActionExecutorFactory actFact = m_executorFactories.get(_wmc.type);
		assert(actFact != null);
		
		ActionExecutor executor = actFact.getActionExecutor();
		if(executor == null) {
			throw new ActionExecutionException("ActionExecutorFactory for " + _wmc.type + " return null");
		}
		
		Action action =  m_component.getMemoryEntry(_wmc.address, Action.class);
		if(executor.accept(action)){

		}
		}

}
