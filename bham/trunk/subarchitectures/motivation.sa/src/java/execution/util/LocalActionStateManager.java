package execution.util;

import java.util.HashMap;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import cast.core.Pair;
import execution.slice.Action;
import execution.slice.ActionExecutionException;
import execution.slice.ActionStatus;
import execution.slice.TriBool;
import execution.util.ActionExecutor.ExecutionCompletionCallback;

/**
 * 
 * Manages interactions with WM for action execution. Should only be constructed
 * with a component that is already running (i.e. in or past the start method
 * 
 * @author nah
 * 
 */
public class LocalActionStateManager extends Thread {

	/**
	 * Wrapper to associate an actor (m_first) with an action (m_second).
	 * 
	 * @author nah
	 * 
	 */
	private static class ExecutorWrapper extends Pair<ActionExecutor, Action> {
		private final WorkingMemoryAddress m_address;

		public ExecutorWrapper(ActionExecutor _a, Action _b,
				WorkingMemoryAddress _address) {
			super(_a, _b);
			m_address = _address;
		}

	}

	private class ExecutionCallback implements ExecutionCompletionCallback {

		private final ExecutorWrapper m_executorWrapper;

		public ExecutionCallback(ExecutorWrapper _actionWrapper) {
			m_executorWrapper = _actionWrapper;
		}

		public void executionComplete(TriBool _success) {
			if (m_component.isRunning()) {
				try {
					m_component.lockComponent();
					actionCompleted(_success, m_executorWrapper.m_address,
							m_executorWrapper.m_second);
					m_component.unlockComponent();

				} catch (CASTException e) {
					m_component.println(e.message);
					e.printStackTrace();
				}
			}
		}
	}

	/**
	 * The component which will do the execution.
	 */
	private final ManagedComponent m_component;

	/**
	 * Queue of things to be executed
	 */
	private Queue<ExecutorWrapper> m_executorQueue;

	/**
	 * Map of classes to factories used to generate the execution objects.
	 */
	private HashMap<String, ActionExecutorFactory> m_executorFactories;

	public LocalActionStateManager(ManagedComponent _component) {

		if (!_component.isRunning()) {
			throw new RuntimeException(
					"LocalActionStateManager should only be constructed with a component that is already running (i.e. in or past the start method");
		}

		m_component = _component;
		start();
	}

	public <Type extends Action> void registerActionType(
			Class<Type> _actionCls, ActionExecutorFactory _executionFactory) {

		// lazy creation
		if (m_executorFactories == null) {
			m_executorFactories = new HashMap<String, ActionExecutorFactory>();
		}

		m_executorFactories.put(CASTUtils.typeName(_actionCls),
				_executionFactory);

		m_component.addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				_actionCls, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						newActionReceived(_wmc);
					}
				});

		m_component.log("registered exection for: " + _actionCls);
	}

	protected void newActionReceived(WorkingMemoryChange _wmc)
			throws ActionExecutionException, DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {
		assert (m_executorFactories != null);
		ActionExecutorFactory actFact = m_executorFactories.get(_wmc.type);
		assert (actFact != null);

		ActionExecutor executor = actFact.getActionExecutor();
		if (executor == null) {
			throw new ActionExecutionException("ActionExecutorFactory for "
					+ _wmc.type + " return null");
		}

		// lazy creation
		if (m_executorQueue == null) {
			m_executorQueue = new ConcurrentLinkedQueue<ExecutorWrapper>();
		}

		Action action = m_component.getMemoryEntry(_wmc.address, Action.class);
		if (executor.accept(action)) {
			actionAccepted(_wmc.address, action);
			m_executorQueue.add(new ExecutorWrapper(executor, action,
					_wmc.address));
		}
	}

	/**
	 * Overwrites action struct to inform other components that action is in
	 * progress.
	 * 
	 * @param _wma
	 * @param _action
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 */
	private void actionAccepted(WorkingMemoryAddress _wma, Action _action)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			ConsistencyException, PermissionException {
		updateAction(_wma, _action, execution.slice.ActionStatus.ACCEPTED,
				TriBool.TRIINDETERMINATE);
	}

	private void actionCompleted(TriBool _success, WorkingMemoryAddress _wma,
			Action _action) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {
		updateAction(_wma, _action, execution.slice.ActionStatus.COMPLETE,
				_success);
	}

	private void updateAction(WorkingMemoryAddress _wma, Action _action,
			ActionStatus _status, TriBool _success)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		_action.status = _status;
		_action.success = _success;
		m_component.overwriteWorkingMemory(_wma, _action);
	}

	@Override
	public void run() {

		while (m_component.isRunning()) {

			m_component.waitForChanges();

			while (m_component.isRunning() && m_executorQueue != null && !m_executorQueue.isEmpty()) {

				try {
					ExecutorWrapper executorWrapper = m_executorQueue.poll();
					assert (executorWrapper != null); // should never be
					ActionExecutor executor = executorWrapper.m_first;
					assert (executor != null);

					if (executor.isBlockingAction()) {
						TriBool executionSuccess = executor.execute();

						// need to lock to protect from other updates
						m_component.lockComponent();
						actionCompleted(executionSuccess,
								executorWrapper.m_address,
								executorWrapper.m_second);
						m_component.unlockComponent();

					} else {
						executor
								.execute(new ExecutionCallback(executorWrapper));
					}
				} catch (CASTException e) {
					m_component.println(e.message);
					e.printStackTrace();
				}

			}
		}
	}
}
