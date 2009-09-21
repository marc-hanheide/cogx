package execution.util;

import java.util.Stack;

import motivation.slice.PlanProxy;
import autogen.Planner.Action;
import autogen.Planner.PlanningTask;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.components.AbstractExecutionManager;

/**
 * A class which manages the execution of a serial plan, using the actions taken directly from the {@link PlanningTask} struct. 
 * 
 * @author nah
 * 
 */
@Deprecated
public class SerialPlanningTaskExecutor extends Thread {

	/**
	 * The component which actually does the work.
	 */
	private final AbstractExecutionManager m_component;

	/**
	 * The address of the {@link PlanningTask} object on WM.
	 */
	private final WorkingMemoryAddress m_planningTaskAddress;

	/**
	 * The address of the {@link PlanProxy} on WM.
	 */
	private final WorkingMemoryAddress m_planProxyAddress;

	private final Stack<Action> m_actionStack;

	private enum ExecutionState {
		PENDING, EXECUTING, HALTED, COMPLETED
	};

	private ExecutionState m_exeState;

	private final PlanningTask m_task;

	private final ActionConverter m_converter;

	private WorkingMemoryChangeReceiver m_stopCallback;

	public SerialPlanningTaskExecutor(AbstractExecutionManager _component,
			WorkingMemoryAddress _planProxyAddress, ActionConverter _converter)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		m_component = _component;
		m_converter = _converter;

		// fetch proxy
		m_planProxyAddress = _planProxyAddress;
		PlanProxy planProxy = m_component.getMemoryEntry(m_planProxyAddress,
				PlanProxy.class);

		m_planningTaskAddress = planProxy.planAddress;
		m_task = m_component.getMemoryEntry(m_planningTaskAddress,
				PlanningTask.class);

		m_actionStack = new Stack<Action>();
		// push actions onto stack
		for (int i = m_task.plan.length - 1; i >= 0; --i) {
			m_actionStack.push(m_task.plan[i]);
		}
		assert (m_actionStack.size() == m_task.plan.length);

		m_exeState = ExecutionState.PENDING;

		// setup callbacks on addresses
		initCallbacks();
	}

	private void initCallbacks() {
		// if either struct is deleted we stop execution

		m_stopCallback = new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				stopExecution();
			}
		};

		m_component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
				m_planningTaskAddress, WorkingMemoryOperation.DELETE),
				m_stopCallback);
		m_component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
				m_planProxyAddress, WorkingMemoryOperation.DELETE),
				m_stopCallback);
	}

	public void startExecution() {
		m_exeState = ExecutionState.EXECUTING;
		start();
	}

	public void stopExecution() throws SubarchitectureComponentException {
		m_exeState = ExecutionState.HALTED;
		cleanup();
	}

	private boolean hasNotBeenStopped() {
		return m_exeState.ordinal() < ExecutionState.HALTED.ordinal();
	}

	private boolean hasMoreActions() {
		return !m_actionStack.isEmpty();
	}

	private Action nextAction() {
		return m_actionStack.pop();
	}

	@Override
	public void run() {

		assert m_exeState == ExecutionState.EXECUTING : "startExecution has not been called";

		try {

			if (!hasMoreActions()) {
				m_component.println("plan was empty, completing execution");
				planComplete();
				return;
			}

			// trigger the first action
			PlannedActionWrapper actionWrapper = triggerNextAction();

			while (m_component.isRunning() && hasNotBeenStopped()) {

				// wait for the component to receive changes
				m_component.waitForChanges();

				// check that we should still do something
				if (m_component.isRunning() && hasNotBeenStopped()) {

					// if one of the changes completed our action
					if (!actionWrapper.isInProgress()) {

						// update state on completion as appropriate, return
						// says whether result of last execution allows
						// execution to continue
						boolean carryOn = actionComplete(actionWrapper);

						// if we can carry on
						if (carryOn) {

							// if more actions
							if (hasMoreActions()) {
								actionWrapper = triggerNextAction();
							}
							// else, plan is complete
							else {
								// plan complete
								planComplete();
							}
						} else {
							// finish
						}
					}

				}

			}

		} catch (CASTException e) {
			e.printStackTrace();
		}

	}

	/**
	 * Clean up saved state. Removes change listeners.
	 * 
	 * @throws SubarchitectureComponentException
	 */
	private void cleanup() throws SubarchitectureComponentException {
		m_component.removeChangeFilter(m_stopCallback);
	}

	/**
	 * Called after all actions complete successfully.
	 * @throws SubarchitectureComponentException 
	 */
	private void planComplete() throws SubarchitectureComponentException {
		
		cleanup();
		
		m_exeState = ExecutionState.COMPLETED;
		m_component.log("plan complete");

		// must do locking in this arbitrary thread
		m_component.lockComponent();
		m_component.deleteFromWorkingMemory(m_planProxyAddress);
		m_component.unlockComponent();
		m_component.log("plan deleted proxy");
	}

	/**
	 * Update state based on action completion. Return true if execution of plan
	 * should continue.
	 * 
	 * @param _actionWrapper
	 * @return
	 */
	private boolean actionComplete(PlannedActionWrapper _actionWrapper) {
		return true;
	}

	/**
	 * @return
	 * @throws CASTException
	 * @throws AlreadyExistsOnWMException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	private PlannedActionWrapper triggerNextAction() throws CASTException,
			AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		Action action = nextAction();
		PlannedActionWrapper actionWrapper = new PlannedActionWrapper(action,
				m_converter.toSystemAction(action));
		m_component.triggerExecution(actionWrapper.execute(), actionWrapper);
		return actionWrapper;
	}

}
