/**
 * 
 */
package execution.components;

import java.util.Arrays;
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
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * A component which will create a plan then execute it. This is a place holder
 * for the later stuff
 * 
 * 
 * @author nah
 * 
 */
public class PrototypePlanExecutor extends ManagedComponent {

	private String m_goal;
	private long m_sleepMillis;
	private Queue<PlanExecutionWrapper> m_plansToExecute;

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
				m_actionStack.push(m_task.plan[i]);
			}
			assert (m_actionStack.size() == m_task.plan.length);
		}

		public boolean hasMoreActions() {
			return !m_actionStack.empty();
		}

		public Action nextAction() {
			return m_actionStack.pop();
		}
	}

	public PrototypePlanExecutor() {
		m_goal = "(forall (?p - place) (= (explored ?p) true))";
		m_sleepMillis = 5000;
	}

	@Override
	protected void configure(Map<String, String> _config) {
		String goal = _config.get("--goal");
		if (goal != null) {
			m_goal = goal;
		}
		log("using goal: " + m_goal);
	}

	@Override
	protected void runComponent() {
		generatePlan();

		PlanExecutionWrapper currentPlan = null;
		Action currentAction = null;

		while (isRunning()) {
			waitForChanges();

			if (m_plansToExecute != null && isRunning()) {

				// if no current pla, get the next one
				if (currentPlan == null && !m_plansToExecute.isEmpty()) {
					currentPlan = m_plansToExecute.poll();
					assert(currentPlan.hasMoreActions());
					currentAction = currentPlan.nextAction();
				}

				// if no current action, get the next one
				else if (currentAction == null) {
					
				}

			}
		}

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
		if (m_plansToExecute == null) {
			m_plansToExecute = new ConcurrentLinkedQueue<PlanExecutionWrapper>();
		}
		m_plansToExecute.add(new PlanExecutionWrapper(_wma, _planningTask));
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

}
