/**
 * 
 */
package execution.components;

import java.util.Map;
import java.util.Queue;
import java.util.Stack;
import java.util.concurrent.ConcurrentLinkedQueue;

import autogen.Planner.Action;
import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.ActionExecutionException;
import execution.slice.TriBool;
import execution.slice.actions.GoToPlace;
import execution.util.ActionMonitor;

/**
 * A component which will create a plan then execute it. This is a place holder
 * for the later stuff.
 * 
 * All actions just stack up for serial execution. Nothing smart is done.
 * 
 * @author nah
 * 
 */
public class PrototypePlanExecutor extends AbstractExecutionManager {

	private String m_goal;
	private long m_sleepMillis;
	private Queue<PlanExecutionWrapper> m_plansToExecute;

	private class PlannedActionWrapper implements ActionMonitor {
		private final Action m_plannedAction;
		private final execution.slice.Action m_systemAction;

		public PlannedActionWrapper(Action _plannedAction,
				execution.slice.Action _systemAction) {
			m_plannedAction = _plannedAction;
			m_systemAction = _systemAction;
		}

		public boolean isInProgress() {
			return m_plannedAction.status == Completion.INPROGRESS;
		}

		public boolean isComplete() {
			return m_plannedAction.status == Completion.SUCCEEDED
					|| m_plannedAction.status == Completion.FAILED;
		}

		/**
		 * Callback which is triggered when the executed action is complete
		 */
		public void actionComplete(execution.slice.Action _action) {
			println("The triggered action is complete... in the future state updates should happen now");
			if (_action.success == TriBool.TRITRUE) {
				m_plannedAction.status = Completion.SUCCEEDED;
			} else {
				m_plannedAction.status = Completion.FAILED;
			}
		}

		/**
		 * Called when execution is to start
		 * 
		 * @return
		 */
		public execution.slice.Action execute() {
			m_plannedAction.status = Completion.INPROGRESS;
			return m_systemAction;
		}

	}

	private class PlanExecutionWrapper {
		private final PlanningTask m_task;
		private final WorkingMemoryAddress m_taskAddress;
		private final Stack<Action> m_actionStack;

		public PlanExecutionWrapper(WorkingMemoryAddress _address,
				PlanningTask _task) {
			m_task = _task;
			m_taskAddress = _address;
			m_actionStack = new Stack<Action>();
			// push actions onto stack
			for (int i = m_task.plan.length - 1; i >= 0; --i) {
				println("stacking action: " + m_task.plan[i].fullName);
				m_actionStack.push(m_task.plan[i]);
			}
			assert (m_actionStack.size() == m_task.plan.length);
		}

		public boolean hasMoreActions() {
			return !m_actionStack.isEmpty();
		}

		public Action nextAction() {
			return m_actionStack.pop();
		}
	}

	public PrototypePlanExecutor() {
		m_goal = "(forall (?p - place) (= (explored ?p) true))";
		m_sleepMillis = 10000;
	}

	@Override
	protected void configure(Map<String, String> _config) {
		String goal = _config.get("--goal");
		if (goal != null) {
			m_goal = goal;
		}
		log("using goal: " + m_goal);
	}

	/**
	 * Does the system specific work of converting a planning action into real
	 * system stuff.
	 * 
	 * @param _plannedAction
	 * @return
	 * @throws CASTException
	 */
	private execution.slice.Action convertToSystemAction(Action _plannedAction)
			throws CASTException {
		if (_plannedAction.name.equals("move")) {
			assert _plannedAction.arguments.length == 2 : "move action arity is expected to be 2";

			GoToPlace act = newActionInstance(GoToPlace.class);
			act.placeID = Long.parseLong(_plannedAction.arguments[1]);
			return act;
		}

		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	/**
	 * 
	 */
	private void generatePlan() {
		sleepComponent(m_sleepMillis);
		String id = newDataID();

		PlanningTask plan = newPlanningTask();
		plan.goal = m_goal;

		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						planGenerated(_wmc.address, getMemoryEntry(
								_wmc.address, PlanningTask.class));

					}
				});

		try {
			addToWorkingMemory(id, plan);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

	private void planGenerated(WorkingMemoryAddress _wma,
			PlanningTask _planningTask) {

		if (_planningTask.planningStatus == Completion.SUCCEEDED) {
			if (m_plansToExecute == null) {
				m_plansToExecute = new ConcurrentLinkedQueue<PlanExecutionWrapper>();
			}
			m_plansToExecute.add(new PlanExecutionWrapper(_wma, _planningTask));
		}
		else {
			println("planning failed: " + _planningTask.planningStatus + " " + _planningTask.goal);
		}
	}

	/**
	 * Create task with non-crashy default values.
	 * 
	 * @return
	 */
	private PlanningTask newPlanningTask() {
		return new PlanningTask(0, null, null, null, Completion.PENDING,
				Completion.PENDING);
	}

	@Override
	protected void runComponent() {
		generatePlan();

		PlanExecutionWrapper currentPlan = null;
		PlannedActionWrapper currentAction = null;

		while (isRunning()) {
			waitForChanges();

			
			try {

				if (m_plansToExecute != null && isRunning()) {

					// TODO hacky illogical structure for now -- really won't
					// work in real situations (e.g. concurrent actions, dodgy
					// plans).

					
					// if no current plan get the next one
					if (currentPlan == null && !m_plansToExecute.isEmpty()) {
					
						currentPlan = m_plansToExecute.poll();

						if (currentPlan.hasMoreActions()) {
							currentAction = nextActionFromPlan(currentPlan);
						} else {
							// in this case we had an empty plan!
							// just print out for the time being
							println("empty plan received, not executing anything.");
							
							println("so let's get a new one");
							generatePlan();
						}

					}
					// if no current action
					// get the next one
					else if (currentAction == null  && !m_plansToExecute.isEmpty()) {
						currentAction = nextActionFromPlan(currentPlan);
					}
					// if an action has just completed
					else if (currentAction != null  && currentAction.isComplete()) {
						if (currentPlan.hasMoreActions()) {
							currentAction = nextActionFromPlan(currentPlan);
						} else {
							println("plan complete!");

							if (!m_plansToExecute.isEmpty()) {
								println("on to the next plan");

								currentPlan = m_plansToExecute.poll();

								if (currentPlan.hasMoreActions()) {
									currentAction = nextActionFromPlan(currentPlan);
								} else {
									// in this case we had an empty plan!
									// just print out for the time being
									println("empty plan received, not executing anything.");
									
									println("so let's get a new one");
									generatePlan();
								}
							} else {
								println("no more plans to execute");
								// reset state
								currentAction = null;
								currentPlan = null;
								
								println("so let's get a new one");
								generatePlan();
								
								
							}
						}
					}
					

					// this will probably break at some point
					if (currentAction != null) {
						if (!currentAction.isInProgress()) {
							triggerExecution(currentAction.execute(),
									currentAction);
						}
					} else {
						println("action is null, not doing anything");
					}
				}
			} catch (CASTException e) {
				println(e.message);
				e.printStackTrace();
			}

		}

	}

	/**
	 * @param _plan
	 * @return
	 * @throws CASTException
	 */
	private PlannedActionWrapper nextActionFromPlan(PlanExecutionWrapper _plan)
			throws CASTException {
		PlannedActionWrapper currentAction;
		Action plannedAction = _plan.nextAction();
		currentAction = new PlannedActionWrapper(plannedAction,
				convertToSystemAction(plannedAction));
		return currentAction;
	}
}
