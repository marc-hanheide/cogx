/**
 * 
 */
package execution.components;

import java.util.Map;

import motivation.slice.PlanProxy;
import autogen.Planner.Action;
import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.ActionExecutionException;
import execution.slice.actions.GoToPlace;
import execution.util.ActionConverter;
import execution.util.SerialPlanExecutor;

/**
 * A component which will create a plan then execute it. This is a place holder
 * for the later stuff.
 * 
 * All actions just stack up for serial execution. Nothing smart is done.
 * 
 * @author nah
 * 
 */
public class PrototypePlanExecutor extends AbstractExecutionManager implements
		ActionConverter {

	private String m_goal;
	private long m_sleepMillis;
	private boolean m_generateOwnPlans;

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

		String selfGenString = _config.get("--self-motivate");
		if (selfGenString != null) {
			m_generateOwnPlans = Boolean.parseBoolean(selfGenString);
		}
		log("generating own plans: " + m_generateOwnPlans);
	}

	@Override
	protected void start() {

		// listen for new PlanProxy structs which trigger execution
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				PlanProxy.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						newPlanProxy(_wmc.address);
					}

				});

		// if generating own plans, listen for execution completions by myself!
		if (m_generateOwnPlans) {
			addChangeFilter(ChangeFilterFactory.createSourceFilter(
					PlanProxy.class, getComponentID(),
					WorkingMemoryOperation.DELETE),
					new WorkingMemoryChangeReceiver() {
						@Override
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) throws CASTException {
							//trigger new plan
							generatePlan();
						}
					});

		}

	}

	private void newPlanProxy(WorkingMemoryAddress _planProxyAddr)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		// create and launch an executor
		new SerialPlanExecutor(this, _planProxyAddr, this).startExecution();
	}

	/**
	 * Does the system specific work of converting a planning action into real
	 * system stuff.
	 * 
	 * @param _plannedAction
	 * @return
	 * @throws CASTException
	 */
	public execution.slice.Action toSystemAction(Action _plannedAction)
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
		
		log("generating new plan with goal: " + m_goal);
		
		String id = newDataID();

		PlanningTask plan = newPlanningTask();
		plan.goal = m_goal;

		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {

						// fetch and create a proxy
						PlanningTask task = getMemoryEntry(_wmc.address,
								PlanningTask.class);
						if (task.planningStatus == Completion.SUCCEEDED) {
							addToWorkingMemory(newDataID(), new PlanProxy(
									_wmc.address));
						}

					}
				});

		try {
			addToWorkingMemory(id, plan);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Create task with non-crashy default values.
	 * 
	 * @return
	 */
	private PlanningTask newPlanningTask() {
		return new PlanningTask(0, null, null, null, null, Completion.PENDING,
				Completion.PENDING);
	}

	@Override
	protected void runComponent() {
		if (m_generateOwnPlans) {
			generatePlan();
		}
	}

}
