/**
 * 
 */
package execution.components;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import motivation.slice.PlanProxy;
import motivation.util.facades.BinderFacade;
import autogen.Planner.Action;
import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.specialentities.RelationProxy;
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
import execution.slice.ActionExecutionException;
import execution.slice.actions.ActiveVisualSearch;
import execution.slice.actions.ExplorePlace;
import execution.slice.actions.GoToPlace;
import execution.util.ActionConverter;
import execution.util.SerialPlanExecutor;
import execution.util.SleepyThread;

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
	private boolean m_kanyeWest;
	private final BinderFacade m_binderFacade;

	// private WorkingMemoryAddress m_lastPlanProxyAddr;

	public PrototypePlanExecutor() {
		m_goal = "(forall (?p - place) (= (explored ?p) true))";
		m_sleepMillis = 10000;
		m_binderFacade = new BinderFacade(this);
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

		m_binderFacade.start();

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

	private void newPlanProxy(WorkingMemoryAddress _planProxyAddr)
			throws SubarchitectureComponentException {
		log("newPlanProxy so creating new plan executor");

		PlanProxy pp = getMemoryEntry(_planProxyAddr, PlanProxy.class);
		PlanningTask pt = getMemoryEntry(pp.planAddress, PlanningTask.class);

		assert (pt.planningStatus == Completion.SUCCEEDED);
		// create and launch an executor
		SerialPlanExecutor executor = new SerialPlanExecutor(this,
				_planProxyAddr, this);
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
			String placeUnionID = plannerLiteralToWMID(_plannedAction.arguments[1]);
			Union placeUnion = m_binderFacade.getUnion(placeUnionID);

			if (placeUnion == null) {
				throw new ActionExecutionException(
						"No union for place union id: " + placeUnionID);
			}
			
			List<FeatureValue> placeIDFeatures = m_binderFacade
					.getFeatureValue(placeUnion, "place_id");
			if (placeIDFeatures.isEmpty()) {
				throw new ActionExecutionException(
						"No place_id features for union id: " + placeUnionID);

			}
			StringValue placeIDString = (StringValue) placeIDFeatures.get(0);
			act.placeID = Long.parseLong(placeIDString.val);
			return act;
		} else if (_plannedAction.name.equals("categorize_room")) {
			assert _plannedAction.arguments.length == 2 : "categorize_room action arity is expected to be 2";
			String roomUnionID = plannerLiteralToWMID(_plannedAction.arguments[1]);
			// ok, we have to do some binder lookups here to find all places
			// that belong to the room:
			// 1. lookup the union for the room
			// 2. find all RelationProxies that have this room union as a source
			// ("contains" relations)
			// 3. read the target of these relations and check if they have a
			// "place_id"
			// 4. add these place_ids to the Action arguments
			Union roomUnion = m_binderFacade.getUnion(roomUnionID);
			Set<Long> placeIDs = new HashSet<Long>();
			log("look at roomUnion:");
			for (Proxy p : roomUnion.includedProxies) {
				// check if it is room

				if (m_binderFacade.getFeatureValue(p, "roomId") != null) {
					Map<WorkingMemoryAddress, RelationProxy> relMap = m_binderFacade
							.findRelationBySrc(p.entityID);
					for (RelationProxy rp : relMap.values()) {
						Proxy placeProxy = m_binderFacade
								.getProxy(((AddressValue) rp.target.alternativeValues[0]).val);
						List<FeatureValue> features = m_binderFacade
								.getFeatureValue(placeProxy, "place_id");
						if (!features.isEmpty()) {
							long placeId = Long
									.parseLong(((StringValue) features.get(0)).val);
							log("  related to this room is place_id " + placeId);
							placeIDs.add(new Long(placeId));
						}

					}
					break;
				}
			}
			ActiveVisualSearch avs = newActionInstance(ActiveVisualSearch.class);
			// TODO: what is expected here???
			avs.placeIDs = new long[placeIDs.size()];
			int count = 0;
			for (Long o : placeIDs)
				avs.placeIDs[count++] = o.longValue();
			return avs;
		}
		else if (_plannedAction.name.equals("explore_place")) {
			return new ExplorePlace();
		}

		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	private String plannerLiteralToWMID(String _string) {
		return _string.substring(_string.indexOf("_") + 1).replace("__", ":");
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
		return new PlanningTask(0, null, null, null, null, Completion.PENDING,
				0, Completion.PENDING, 0);
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
