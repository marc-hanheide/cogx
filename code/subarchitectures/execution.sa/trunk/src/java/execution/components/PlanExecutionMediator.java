/**
 * 
 */
package execution.components;

import java.util.Map;

import motivation.slice.PlanProxy;
import autogen.Planner.Completion;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
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
import execution.util.ActionConverter;
import execution.util.SerialPlanExecutor;
import execution.util.SleepyThread;

/**
 * Converts planner actions into things the system can act on then executes
 * these things. This is the main execution interface between the planner and
 * the rest of the system. A new instance of {@link SerialPlanExecutor} is
 * created for each plan to be executed. This object handles the actual
 * interactions with the rest of the system.
 * 
 * @author nah
 * 
 */
public abstract class PlanExecutionMediator extends AbstractExecutionManager {

	private String m_goal;
	private long m_sleepMillis;
	private boolean m_generateOwnPlans;
	private boolean m_kanyeWest;

	// private WorkingMemoryAddress m_lastPlanProxyAddr;

	public PlanExecutionMediator() {
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

		String interruptSelfString = _config.get("--interrupt");
		if (interruptSelfString != null) {
			m_kanyeWest = Boolean.parseBoolean(interruptSelfString);
		}
		log("interrupting own execution: " + m_kanyeWest);

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
							// trigger new plan
							println("received delete change for generate plan");
							new PlanGenerator().start();
						}
					});

		}

	}

	public abstract ActionConverter getActionConverter();

	private void newPlanProxy(WorkingMemoryAddress _planProxyAddr)
			throws SubarchitectureComponentException {
		log("newPlanProxy so creating new plan executor");

		PlanProxy pp = getMemoryEntry(_planProxyAddr, PlanProxy.class);
		PlanningTask pt = getMemoryEntry(pp.planAddress, PlanningTask.class);

		assert (pt.planningStatus == Completion.SUCCEEDED);

		// create and launch an executor
		SerialPlanExecutor executor = new SerialPlanExecutor(this,
				_planProxyAddr, getActionConverter());
		executor.startExecution();

		if (m_generateOwnPlans && m_kanyeWest) {
			new SelfInterrupter(_planProxyAddr).start();
		}
	}

	private class SelfInterrupter extends SleepyThread {
		WorkingMemoryAddress m_planProxyAddress;

		public SelfInterrupter(WorkingMemoryAddress _ppa) {
			super(2000);
			m_planProxyAddress = _ppa;
		}

		@Override
		public void doSomething() {
			try {
				println("interrupting at: "
						+ CASTUtils.toString(m_planProxyAddress));
				lockComponent();
				deleteFromWorkingMemory(m_planProxyAddress);
				unlockComponent();
				println("deleted plan proxy");
			} catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			} catch (PermissionException e) {
				e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				e.printStackTrace();
			}
		}
	}

	private class PlanGenerator extends SleepyThread {

		public PlanGenerator() {
			super(m_sleepMillis);
		}

		@Override
		protected void doSomething() {
			generatePlan();
		}
	}

	/**
	 * 
	 */
	private void generatePlan() {

		log("generating new plan with goal: " + m_goal);

		String id = newDataID();

		PlanningTask plan = newPlanningTask();
		plan.goals = new Goal[1];
		plan.goals[0].goalString = m_goal;
		plan.goals[0].importance = -1;
		plan.goals[0].isInPlan = false;

		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {

						// fetch and create a proxy
						PlanningTask task = getMemoryEntry(_wmc.address,
								PlanningTask.class);
						if (task.planningStatus == Completion.SUCCEEDED) {
							println("planning succeeded, adding proxy");
							addToWorkingMemory(newDataID(), new PlanProxy(
									_wmc.address));
							removeChangeFilter(this);
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
		return new PlanningTask(0, null, true, null, 0, null, Completion.PENDING,
                                0, Completion.PENDING, 0, null);
	}

	@Override
	protected void runComponent() {
		if (m_generateOwnPlans) {
			new PlanGenerator().start();
		}
	}

	public static void main(String[] args) {
		String testCase = "place_3__f";
		System.out.println(testCase.substring(testCase.indexOf("_") + 1)
				.replace("__", ":"));

	}

}
