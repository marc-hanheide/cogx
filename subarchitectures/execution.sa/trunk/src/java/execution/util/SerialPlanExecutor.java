package execution.util;

import motivation.slice.PlanProxy;
import autogen.Planner.Action;
import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.experimentation.StopWatch;
import execution.components.AbstractExecutionManager;
import execution.slice.TriBool;

/**
 * A class which manages the execution of a serial plan, using the actions
 * written separately to WM. There is probably some redundancy here, but the
 * interactions with the planner are not quite the most obvious things to track
 * at the moment.
 * 
 * @author nah
 * 
 */
public class SerialPlanExecutor extends Thread {

	/**
	 * The component which actually does the work.
	 */
	private final AbstractExecutionManager m_component;

	/**
	 * The address of the {@link PlanningTask} object on WM.
	 */
	private WorkingMemoryAddress m_planningTaskAddress;

	/**
	 * The address of the {@link PlanProxy} on WM.
	 */
	private final WorkingMemoryAddress m_planProxyAddress;

	/**
	 * The address where actions are written on WM.
	 */
	private WorkingMemoryAddress m_actionAddress;

	/***
	 * The action being executed
	 * 
	 * @author nah
	 * 
	 */
	private Action m_currentAction;

	private enum ExecutionState {
		PENDING, EXECUTING, HALTED, COMPLETED
	};

	private ExecutionState m_exeState;

	private PlanningTask m_task;

	private final ActionConverter m_converter;

	private WorkingMemoryChangeReceiver m_stopCallback;

	private WorkingMemoryChangeReceiver m_planChangedCallback;

	private WorkingMemoryChangeReceiver m_actionChangedCallback;

	private StopWatch stopWatch;



	public SerialPlanExecutor(AbstractExecutionManager _component,
			WorkingMemoryAddress _planProxyAddress, ActionConverter _converter)
			throws SubarchitectureComponentException {

		m_exeState = ExecutionState.PENDING;

		stopWatch = new StopWatch("SerialPlanstExecuter");

		m_component = _component;
		m_converter = _converter;

		// fetch proxy
		m_planProxyAddress = _planProxyAddress;
		
		

	}

	
	private void initCallbacks() {
		// if either struct is deleted we stop execution
		m_stopCallback = new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				m_component.log("something was deleted, stopping exe: "
						+ CASTUtils.toString(_wmc));
				stopExecution();
			}
		};

		m_component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
				m_planningTaskAddress, WorkingMemoryOperation.DELETE),
				m_stopCallback);
		
		
		m_component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
				m_planProxyAddress, WorkingMemoryOperation.DELETE),
				m_stopCallback);

		// if plan is overwritten it could be for us to stop too
		m_planChangedCallback = new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				planChanged(m_component.getMemoryEntry(_wmc.address,
						PlanningTask.class));
			}
		};
		m_component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
				m_planningTaskAddress, WorkingMemoryOperation.OVERWRITE),
				m_planChangedCallback);

		// also, if the action is overwritten then it could be a signal to
		// change action
		m_actionChangedCallback = new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				if (_wmc.src.equals(m_component.getComponentID())) {
					m_component.log("	 by me");
				} else {
					m_component.log("action changed by " + _wmc.src);
				}
			}
		};

		m_component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
				m_actionAddress, WorkingMemoryOperation.OVERWRITE),
				m_actionChangedCallback);

	}

	private void planChanged(PlanningTask _planningTask)
			throws SubarchitectureComponentException {
		// if plan succeeded then we're done
		m_component.log("plan changed");

		// execution deemed complete by the planner
		if (_planningTask.executionStatus == Completion.SUCCEEDED) {
			stopExecution();
			planComplete(ExecutionState.COMPLETED);
			m_component.log("and read that plan complete");
		}
		// execution deemed failed by the planner
		else if (_planningTask.executionStatus == Completion.FAILED) {
			stopExecution();
			planComplete(ExecutionState.HALTED);
			m_component.log("and read that planner thinks that execution has failed (probably too many failures)");
		}
		// execution deemed complete by the planner
		else if (_planningTask.planningStatus == Completion.FAILED) {
			stopExecution();
			planComplete(ExecutionState.HALTED);
			m_component.log("and read that replanning has failed");
		}

	}

	public void startExecution() throws SubarchitectureComponentException {

		stopWatch.tic();

		PlanProxy planProxy = m_component.getMemoryEntry(m_planProxyAddress,
				PlanProxy.class);

		m_planningTaskAddress = planProxy.planAddress;

		m_component.log("executing plan at: "
				+ CASTUtils.toString(m_planningTaskAddress));
		m_task = m_component.getMemoryEntry(m_planningTaskAddress,
				PlanningTask.class);

		assert m_task.planningStatus == Completion.SUCCEEDED : "can't execute a non-succeeded plan: "
				+ m_task.executionStatus;

		m_actionAddress = new WorkingMemoryAddress(m_task.firstActionID,
				m_planningTaskAddress.subarchitecture);

		if (m_task.firstActionID == null || m_task.firstActionID.isEmpty()) {
			m_component.log("plan was empty, completing");
			planComplete(ExecutionState.COMPLETED);
			stopWatch.toc("execution complete: " + CASTUtils.toString(m_planningTaskAddress));
			return;
		}

		m_currentAction = m_component.getMemoryEntry(m_actionAddress,
				Action.class);

		// setup callbacks on addresses
		initCallbacks();

		// m_component.log("running from state: " + m_exeState);
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

	@Override
	public void run() {

		if (m_exeState != ExecutionState.EXECUTING) {
			m_component.log("not executing run loop");
			stopWatch.toc("execution complete: " + CASTUtils.toString(m_planningTaskAddress));
			return;
		}

		PlannedActionWrapper actionWrapper = null;
		try {

			// trigger the first action
			actionWrapper = triggerNextAction(null);
			assert actionWrapper != null : "first action should not be null";

			while (m_component.isRunning() && hasNotBeenStopped()) {

				// wait for the component to receive changes
				m_component.waitForChanges();
				
				m_component.lockComponent();
				
				while (m_component.isPaused()) {
					m_component.log("we are paused... let's wait");
					try {
						sleep(1000);
					} catch (InterruptedException e) {
						m_component.getLogger().warn("interrupted:", e);
						break;
					}
				}
					
				
				// check that we should still do something
				if (m_component.isRunning()) {

					// if we've been explicitly stopped
					if (m_exeState == ExecutionState.HALTED) {
						// if something is happening
						if (actionWrapper.isInProgress()) {
							m_component.log("stopping action in progress");
							m_component.stopExecution(actionWrapper
									.getActionAddress());
						}
					} else if (hasNotBeenStopped()) {
						// if one of the changes completed our action
						if (!actionWrapper.isInProgress()) {
							// and we haven't told the planner its complete
							if (!actionWrapper.haveSentCompletionSignal()) {
								
								// update state on completion as appropriate,
								// return
								// says whether result of last execution allows
								// execution to continue
								signalActionComplete(actionWrapper);
							} else {
								
								// try to trigger the next action. if nothing is
								// triggered then actionWrapper doesn't change.
								actionWrapper = triggerNextAction(actionWrapper);
							}
						}
					}
				}
			
				m_component.unlockComponent();
				
			}
		} catch (CASTException e) {
			m_component.logException(e);
			try {
				planComplete(ExecutionState.COMPLETED);
			} catch (SubarchitectureComponentException e2) {
				m_component.logException(e2);
			}

			if (actionWrapper != null) {
				m_component.getLogger().warn("stopping action in progress");
				try {
					m_component.stopExecution(actionWrapper.getActionAddress());
				} catch (CASTException e1) {
					m_component.logException(e1);
				}
			}
			
			m_component.unlockComponent();

		}
		stopWatch.toc("execution complete: " + CASTUtils.toString(m_planningTaskAddress));
	}

	/**
	 * Clean up saved state. Removes change listeners.
	 * 
	 * @throws SubarchitectureComponentException
	 */
	private void cleanup() throws SubarchitectureComponentException {
		// callback could be null if plan was empty
		if (m_stopCallback != null) {
			m_component.removeChangeFilter(m_stopCallback);
			m_component.removeChangeFilter(m_planChangedCallback);
			m_component.removeChangeFilter(m_actionChangedCallback);
		}
	}

	/**
	 * Called after all actions complete successfully.
	 * 
	 * @throws SubarchitectureComponentException
	 */
	private void planComplete(ExecutionState _state)
			throws SubarchitectureComponentException {

		cleanup();

		m_exeState = _state;
		m_component.log("plan complete");
		try {
			m_component.deleteFromWorkingMemory(m_planProxyAddress);
			m_component.log("executor deleted proxy");
		} catch (DoesNotExistOnWMException e) {
			m_component
					.println("plan proxy was deleted before we could remove it: "
							+ e.getMessage());
		}
	}

	/**
	 * Update state based on action completion. Return true if execution of plan
	 * should continue.
	 * 
	 * @param _actionWrapper
	 */
	private void signalActionComplete(PlannedActionWrapper _actionWrapper)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		if (_actionWrapper.wasSuccessful() == TriBool.TRITRUE) {
			m_currentAction.status = Completion.SUCCEEDED;
		} else {
			m_currentAction.status = Completion.FAILED;
		}
		m_component.overwriteWorkingMemory(m_actionAddress, m_currentAction);
		_actionWrapper.sentCompletionSignal();
		m_component.log("signalled planner that action was completed.");
	}

	/**
	 * Tries to trigger the next action. Will only do so if {@link Action}
	 * .status is PENDING.
	 * 
	 * @return the action wrapper for the new action, or the previous action if
	 *         nothing was triggered.
	 * @throws CASTException
	 * @throws AlreadyExistsOnWMException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	private PlannedActionWrapper triggerNextAction(
			PlannedActionWrapper _previousAction) throws CASTException,
			AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		Action action = readAction();
		PlannedActionWrapper actionWrapper = _previousAction;
		m_component.log("act status: " + action.status);
		if (action.status == Completion.PENDING) {
			actionWrapper = new PlannedActionWrapper(action, m_converter
					.toSystemAction(action));
			WorkingMemoryAddress actionAddr = m_component.triggerExecution(
					actionWrapper.execute(), actionWrapper);
			actionWrapper.setActionAddress(actionAddr);
		}
		return actionWrapper;

	}

	private Action readAction() throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		return m_component.getMemoryEntry(m_actionAddress, Action.class);
	}

}
