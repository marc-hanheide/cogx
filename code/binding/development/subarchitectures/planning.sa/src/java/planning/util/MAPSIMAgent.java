/**
 * 
 */
package planning.util;

import java.util.ArrayList;
import java.util.Properties;

import org.omg.CORBA.COMM_FAILURE;
import org.omg.CORBA.ORB;

import planning.autogen.PlanningStateLists;
import Planner.CCPState;
import Planner.Command;
import Planner.ExecutionState;
import Planner.Fact;
import Planner.Failure;
import Planner.ObjectDeclaration;
import Planner.PlannerServer;
import Planner.PlanningState;
import Planner.PlanningTask;
import Planner.PlanningTaskHolder;
import balt.corba.impl.RemoteConnectionManager;

/**
 * This is an object that wraps up planner interactions into a single object.
 * Manages calls to {@link PlannerServer}.
 * 
 * @author Nick Hawes, Michael Brenner
 */
public class MAPSIMAgent {

	// // may need this later
	// private ObjectDeclaration m_actor;

	private PlannerServer m_planner;

	private CCPState m_status;

	private PlanningTask m_task;

	private TemporaryPlanningState m_additionalState;

	/**
	 * Create a new MAPSIM agent. Currently this should be done via
	 * {@link MAPSIMAgentFactory} to ensure the correct parameters are set.
	 * 
	 * @param _domain
	 * @param _domainFile
	 * @param _agentName
	 * @param _log
	 */
	MAPSIMAgent(String _domain, String _domainFile, String _agentName,
			boolean _log) {
		m_task = new PlanningTask("", _agentName, null, null, null, _domain,
				_domainFile);
		m_planner = PlannerServerFactory.getPlannerServer();
		// m_actor = new ObjectDeclaration(_agentName, "robot");
		m_additionalState = new TemporaryPlanningState();
	}

	/**
	 * Get the current state as stored by the planner.
	 * 
	 * @return
	 */
	public Fact[] currentFacts() {
		assert (m_task != null);
		assert (m_task.task_id != null);
		return m_planner.current_task_state(m_task.task_id).facts;
	}
	/**
	 * Get the current state as stored by the planner.
	 * 
	 * @return
	 */
	public ObjectDeclaration[] currentObjects() {
		assert (m_task != null);
		assert (m_task.task_id != null);
		return m_planner.current_task_state(m_task.task_id).objects;
	}

	/**
	 * Get the changes that would be expected if the input command was executed.
	 * 
	 * @param _cmd
	 * @return
	 */
	public Fact[] expectedChange(Command _cmd) {
		assert (m_task != null);
		assert (m_task.task_id != null);
		return m_planner.expected_changes(m_task.task_id, _cmd);

	}

	/**
	 * Get the next step to take from the plan. If there are multiple possible
	 * steps, this just picks the first.
	 * 
	 * @return The next step.
	 */
	public Command getNextStep() {
		assert (m_task != null);
		assert (m_task.task_id != null);

		// get executable steps from the current plan
		Command[] possibleCommands = m_planner
				.next_executable_plan_steps(m_task.task_id);

		for (Command command : possibleCommands) {
			System.out.println("command: " + PlanningUtils.toString(command));
		}

		assert (possibleCommands.length > 0);

		// // just pick last
		// return possibleCommands[possibleCommands.length - 1];
		// just pick first
		return possibleCommands[0];
	}

	/**
	 * Get the next possible steps from the plan.
	 * 
	 * @return The next steps.
	 */
	public Command[] getNextSteps() {
		assert (m_task != null);
		assert (m_task.task_id != null);

		// get executable steps from the current plan
		Command[] possibleCommands = m_planner
				.next_executable_plan_steps(m_task.task_id);

		assert (possibleCommands.length > 0);

		// just pick first
		return possibleCommands;
	}

	/**
	 * Is the current goal achievable.
	 * 
	 * @return
	 */
	public boolean goalAcheivable() {
		assert (m_status != null);
		return m_status.execution_state == ExecutionState.GOAL_ACHIEVABLE;
	}

	/**
	 * Is the current goal achieved.
	 * 
	 * @return
	 */
	public boolean goalAchieved() {
		assert (m_status != null);
		return m_status.execution_state == ExecutionState.GOAL_ACHIEVED;
	}

	/**
	 * Has replanning been triggered.
	 * 
	 * @return
	 */
	public boolean replanningTriggered() {
		assert (m_status != null);
		return m_status.planning_state == PlanningState.CHANGED_PLAN;
	}

	/**
	 * Set the goal for the agent. Must be called before start planning.
	 * 
	 * @param _goal
	 */
	public void setGoal(String _goal) {
		assert (m_task.goals == null);
		m_task.goals = new String[]{_goal};
	}

	// private <T> T[] join(T[] _a, T[] _b) {
	//
	// ArrayList<T> r = new ArrayList<T>();
	// for (T a : _a) {
	// r.add(a);
	// }
	// for (T b : _b) {
	// r.add(b);
	// }
	// return r.toArray(_a);
	// }

	private void setState(PlanningTask _task, ObjectDeclaration[] _objs,
			Fact[] _facts) {

		ArrayList<ObjectDeclaration> objs = new ArrayList<ObjectDeclaration>(
				m_additionalState.m_objectList);
		for (ObjectDeclaration od : _objs) {
			objs.add(od);
		}
		_task.objects = new ObjectDeclaration[objs.size()];
		objs.toArray(_task.objects);

		ArrayList<Fact> facts = new ArrayList<Fact>(
				m_additionalState.m_factList);
		for (Fact fct : _facts) {
			facts.add(fct);
		}
		_task.facts = new Fact[facts.size()];
		facts.toArray(_task.facts);

	}

	/**
	 * Set the intitial state of the world, all operations are performed
	 * relative to this. Must be called before startPlanning.
	 * 
	 * @param _planningState
	 */
	public void setInitialState(ObjectDeclaration[] _objs, Fact[] _facts) {
		assert (m_task.objects == null);
		assert (m_task.facts == null);
		setState(m_task, _objs, _facts);
	}

	/**
	 * Start the planning process.
	 * 
	 * @throws PlanningException
	 */
	public void startPlanning() throws PlanningException {
		assert (m_task.task_id.equals(""));

		try {
			PlanningTaskHolder pth = new PlanningTaskHolder(m_task);

			System.out.println("task: " + PlanningUtils.toString(m_task));

			// keep trying through connection failure
			boolean done = false;
			while (!done) {
				try {
					m_planner.new_task(pth);
					done = true;
				}
				catch (COMM_FAILURE e) {
					System.err
							.println("MAPSIMAgent.startPlanning: CORBA COMM_FAILURE, trying again");
					try {
						Thread.sleep(1000);
					}
					catch (InterruptedException e1) {
						e1.printStackTrace();
					}

					PlannerServerFactory.shutdown();
					m_planner = PlannerServerFactory.getPlannerServer();

				}
			}

			m_task = pth.value;

			assert (!m_task.task_id.equals(""));

			callPlanner();
		}
		catch (org.omg.CORBA.UNKNOWN e) {
			throw new PlanningException("Failure in planner. Check output.", e);
		}
	}

	/**
	 * Inform MAPSIM that a step has been taken. Also provide the new world
	 * state that has resulted from this step.
	 * 
	 * @param _cmd
	 * @param _succeess
	 * @param _resultingFacts
	 * @param _del
	 * @param _add
	 * @throws PlanningException
	 */
	public void stepTaken(Command _cmd, boolean _succeess,
			ObjectDeclaration[] _resultingObjects, Fact[] _resultingFacts,
			PlanningStateLists _add, PlanningStateLists _del)
			throws PlanningException {
		assert (m_task != null);
		assert (m_task.task_id != null);
		assert (_cmd != null);
		assert (_resultingFacts != null);
		assert (_resultingObjects != null);

		// inform planner that the cmd was executed
		m_planner.command_was_executed(m_task.task_id, _cmd, _succeess);

		updateAdditionalState(_add, _del);

		System.out.println("MAPSIMAgent.stepTaken(): " + m_additionalState);

		// now update the task with the new state
		setState(m_task, _resultingObjects, _resultingFacts);

		System.out.println("State sent to planner:");
		System.out.println("Objects:");
		for (ObjectDeclaration obj : m_task.objects) {
			System.out.println(PlanningUtils.toString(obj));
		}
		System.out.println("Facts:");
		for (Fact fct : m_task.facts) {
			System.out.println(PlanningUtils.toString(fct));
		}

		m_planner.change_task(m_task, true);

		assert (!m_task.task_id.equals(""));
		// and now rerun planner
		callPlanner();
	}

	private void updateAdditionalState(PlanningStateLists _add,
			PlanningStateLists _del) {
		// update objects

		// just add additions blindly
		m_additionalState.extend(_add);

		// for each object stored in additional state
		for (ObjectDeclaration toDeleteObj : _del.m_objects) {
			// for each post-action removal
			for (ObjectDeclaration additionalObj : m_additionalState.m_objectList) {
				if (PlanningUtils.equals(toDeleteObj, additionalObj)) {
					System.out.println("removing from additional list: "
							+ toDeleteObj);
					m_additionalState.m_objectList.remove(additionalObj);
					break;
				}
			}
		}

		// for each object stored in additional state
		for (Fact toDeleteFact : _del.m_facts) {
			// for each post-action removal

			for (Fact additionalFact : m_additionalState.m_factList) {

				if (PlanningUtils.equals(toDeleteFact, additionalFact)) {
					System.out.println("removing from additional list: "
							+ toDeleteFact);
					m_additionalState.m_factList.remove(additionalFact);
					break;
				}
			}
		}

	}

	/**
	 * Inform MAPSIM that a step has been taken successfully. Also provide the
	 * new world state that has resulted from this step.
	 * 
	 * @param _cmd
	 * @param _succeess
	 * @param _resultingFacts
	 * @throws PlanningException
	 */
	public void stepTaken(Command _cmd, ObjectDeclaration[] _resultingObjects,
			Fact[] _resultingFacts) throws PlanningException {
		stepTaken(_cmd, true, _resultingObjects, _resultingFacts,
				new PlanningStateLists(new ObjectDeclaration[0], new Fact[0]),
				new PlanningStateLists(new ObjectDeclaration[0], new Fact[0]));
	}

	/**
	 * Trigger continual planning and update status.
	 * 
	 * @throws PlanningException
	 */
	private void callPlanner() throws PlanningException {
		try {
			m_status = m_planner.continual_planning(m_task.task_id);
		}
		catch (Failure f) {
			throw new PlanningException("Failure during first call to planner",
					f);
		}
	}

	/**
	 * @param args
	 * @throws Failure
	 * @throws PlanningException
	 */
	public static void main(String[] args) throws Failure, PlanningException {

		testMAPSIM();
	}

	public static boolean testMAPSIM() {
		try {
			// Setup the ugly CORBA bits
			Properties props = new Properties();
			props.put("org.omg.CORBA.ORBInitialPort", "1050");
			props.put("org.omg.CORBA.ORBInitialHost", "localhost");
			// initialise orb
			ORB.init(new String[]{}, props);
			// initialise balt
			RemoteConnectionManager.init(new String[]{}, props);

			// tell the planner server where the server executable is
			PlannerServerFactory
					.setPlannerExecutablePath("./subarchitectures/planning.sa/src/python/mapsim/planning/server.py");

			// server.self_test();
			// System.out.println("self_test done");

			// ContinualPlanningProcess cpp = setupClarificationProblem();
			// ContinualPlanningProcess cpp = setupGridProblem();
			// ContinualPlanningProcess cpp = setupMGridProblem();
			MAPSIMAgent cpp = setupPlaymateProblem();

			cpp.startPlanning();

			// System.out.println("plan: " +
			// Arrays.toString(cpp.getPlan().nodes));
			//
			// // System.exit(1);
			//
			// tum te tum, could run forever
			while (true) {
				//
				// System.out.println("Current state: "
				// + Arrays.toString(cpp.getCurrentState().m_facts));
				//
				if (cpp.goalAchieved()) {
					System.out.println("Goal Achieved!");
					// normal users wouldn't usually do this
					PlannerServerFactory.shutdown();

					return true;
				}
				if (cpp.replanningTriggered()) {
					System.out.println("Replanning triggered!");
				}
				else if (!cpp.goalAcheivable()) {
					System.out.println("Plan no longer achieves goal!");
					// normal users wouldn't usually do this
					PlannerServerFactory.shutdown();

					return false;
				}

				//
				Command nextStep = cpp.getNextStep();

				System.out.println("taking step: "
						+ PlanningUtils.toString(nextStep));

				Fact[] newState = PlanningUtils.changeState(cpp.currentFacts(),
						cpp.expectedChange(nextStep));
				cpp.stepTaken(nextStep, cpp.currentObjects(), newState);

				// step taking fails and we replan then loop forever
				// cpp.stepTaken(nextStep, false, cpp.currentState());
			}
		}
		catch (Throwable t) {
			t.printStackTrace();
		}

		// normal users wouldn't usually do this
		PlannerServerFactory.shutdown();

		return false;
	}

	// public static ContinualPlanner setupClarificationProblem() {
	// // try a grid world problem... this is from the mapsim code
	// ObjectDeclaration objects[] = {
	// new ObjectDeclaration("MrChips", "agent"),
	// new ObjectDeclaration("Michael", "agent"),
	// new ObjectDeclaration("MrChips", "planning_agent"),
	// new ObjectDeclaration("vp1", "vision_proxy"),
	// new ObjectDeclaration("vp2", "vision_proxy"),
	// new ObjectDeclaration("vp3", "vision_proxy"),
	// new ObjectDeclaration("lp1", "language_proxy")};
	//
	// String[] facts = {"in_feature_domain red colour",
	// "in_feature_domain blue colour",
	// "in_feature_domain green colour",
	// "in_feature_domain mug object_class",
	// "has_access_to_modality Michael vision",
	// "has_access_to_modality Michael language",
	// "has_access_to_modality vision_sa vision",
	// "has_access_to_modality vp1 vision",
	// "distinctive_feature colour vp1",
	// "distinctive_feature object_class vp1",
	// "kd__feature_val vision_sa colour vp1",
	// "kd__feature_val vision_sa object_class vp1",
	// "in_binding_candidates vp1 lp1",
	// "has_access_to_modality vp2 vision",
	// "distinctive_feature colour vp2",
	// "distinctive_feature object_class vp2",
	// "kd__feature_val vision_sa colour vp2",
	// "kd__feature_val vision_sa object_class vp2",
	// "in_binding_candidates vp2 lp1",
	// "has_access_to_modality vp3 vision",
	// "distinctive_feature colour vp3",
	// "distinctive_feature object_class vp3",
	// "kd__feature_val vision_sa colour vp3",
	// "kd__feature_val vision_sa object_class vp3",
	// "in_binding_candidates vp3 lp1",
	// "has_access_to_modality lp1 language",
	// "distinctive_feature colour lp1",
	// "distinctive_feature object_class lp1",
	// "kd__feature_val Michael colour lp1",
	// "kd__feature_val Michael object_class lp1",
	// "feature_val object_class vp1 : mug",
	// "feature_val object_class vp2 : mug",
	// "feature_val object_class vp3 : mug",
	// "feature_val object_class lp1 : mug",
	// "feature_val colour lp1 : blue"};
	//
	// String goal = "(exists (?p - binding_proxy) (is_bound lp1 ?p))";
	// String domainFile =
	// "./subarchitectures/planning.sa/src/python/mapsim/domains/clarification/mapl_files/domain.mapl";
	//
	// ContinualPlanner cpp = new ContinualPlanner(domainFile, true);
	// cpp.setInitialState(new PlanningState(objects, facts));
	// cpp.setGoal(goal);
	// return cpp;
	// }

	// public static ContinualPlanner setupGridProblem() {
	// // try a grid world problem... this is from the mapsim code
	// ObjectDeclaration objects[] = {
	// new ObjectDeclaration("c11", "gridcell"),
	// new ObjectDeclaration("c00", "gridcell"),
	// new ObjectDeclaration("agt0", "gridcontent"),
	// new ObjectDeclaration("c01", "gridcell"),
	// new ObjectDeclaration("c10", "gridcell"),
	// new ObjectDeclaration("agt0", "planning_agent")};
	//
	// String[] facts = {"connected c00 c01", "connected c00 c10",
	// "connected c01 c00", "connected c01 c11", "connected c10 c00",
	// "connected c10 c11", "connected c11 c01", "connected c11 c10",
	//
	// // ;; dynamic facts
	// "occupant c00 empty", "occupant c01 empty",
	// "occupant c10 empty", "occupant c11 agt0",};
	//
	// String goal = "(and (occupant c00 agt0))";
	// String domainFile =
	// "./subarchitectures/planning.sa/src/python/mapsim/domains/gridworld/mapl_files/gridworld-static-domain.mapl";
	//
	// ContinualPlanner cpp = new ContinualPlanner(domainFile, true);
	// cpp.setInitialState(new PlanningState(objects, facts));
	// cpp.setGoal(goal);
	// return cpp;
	// }

	// public static ContinualPlanner setupMGridProblem() {
	// // try a grid world problem... this is from the mapsim code
	// ObjectDeclaration objects[] = {
	// // A 3X3 grid world...
	// new ObjectDeclaration("c0", "gridcell"),
	// new ObjectDeclaration("c1", "gridcell"),
	// new ObjectDeclaration("c2", "gridcell"),
	// new ObjectDeclaration("c3", "gridcell"),
	// new ObjectDeclaration("c4", "gridcell"),
	// new ObjectDeclaration("c5", "gridcell"),
	// new ObjectDeclaration("c6", "gridcell"),
	// new ObjectDeclaration("c7", "gridcell"),
	// new ObjectDeclaration("c8", "gridcell"),
	// // Then some agents as well..
	// new ObjectDeclaration("agt0", "gridcontent"),
	// new ObjectDeclaration("agt1", "gridcontent"),
	// new ObjectDeclaration("agt0", "planning_agent"),
	// new ObjectDeclaration("agt1", "planning_agent")};
	//
	// String[] facts = {
	// // Establish the connectivity between the different cells...
	// "connected c0 c1", "connected c0 c3", "connected c1 c0",
	// "connected c1 c2", "connected c1 c4", "connected c2 c1",
	// "connected c2 c5", "connected c3 c0",
	// "connected c3 c4",
	// "connected c3 c6",
	// "connected c4 c1",
	// "connected c4 c3",
	// "connected c4 c5",
	// "connected c4 c7",
	// "connected c5 c2",
	// "connected c5 c4",
	// "connected c5 c8",
	// "connected c6 c3",
	// "connected c6 c7",
	// "connected c7 c4",
	// "connected c7 c6",
	// "connected c7 c8",
	// "connected c8 c5",
	// "connected c8 c7",
	//
	// // Facts about the world that could change over time (well
	// // the obstacles cannot be moved but anyway it makes sense
	// // to leave it here to give an initial picture of the
	// // world...
	// "occupant c0 empty", "occupant c1 obstacle",
	// "occupant c2 agt1", "occupant c3 empty", "occupant c4 empty",
	// "occupant c5 empty", "occupant c6 agt0",
	// "occupant c7 obstacle", "occupant c8 empty",};
	//
	// String goal = "(and ( occupant c2 agt0 ) ( occupant c6 agt1 ) )";
	// String domainFile =
	// "./subarchitectures/planning.sa/src/python/mapsim/domains/gridworld/mapl_files/gridworld-static-domain.mapl";
	//
	// ContinualPlanner cpp = new ContinualPlanner(domainFile, true);
	// cpp.setInitialState(new PlanningState(objects, facts));
	// cpp.setGoal(goal);
	// return cpp;
	// }

	// /**
	// * Mohan: And while we are at it, we may as well get started on the
	// problem
	// * with visual regions and their associated planning...
	// */
	// public static ContinualPlanner setupVisRegionProblem() {
	// ObjectDeclaration objects[] = {
	// // Let us also have three visual regions...
	// new ObjectDeclaration("vr0", "visRegion"),
	// new ObjectDeclaration("vr1", "visRegion"),
	// new ObjectDeclaration("vr2", "visRegion")};
	//
	// String[] facts = {
	// // Color properties of the regions...
	// "colorProp-val vr0 red",
	// "colorProp-val vr1 blue",
	// "colorProp-val vr2 blue",
	// // Sift properties of the regions...
	// "objectType-val vr0 picture", "objectType-val vr1 mug",
	// "objectType-val vr2 cuboid",
	// // Shape properties of the regions...
	// "shapeProp-val vr0 circular", "shapeProp-val vr1 circular",
	// "shapeProp-val vr2 squared",};
	//
	// String goal = "(and (exists (?vr - visRegion) (and ( colorProp-val ?vr
	// blue ) ( containsObject ?vr mug ) ( containsShape ?vr circular ) ) ) )";
	//
	// String domainFile =
	// "./subarchitectures/planning.sa/src/python/mapsim/domains/blocksworld/mapl_files/blocksworld-domain.mapl";
	//
	// ContinualPlanner cpp = new ContinualPlanner(domainFile, true);
	// cpp.setInitialState(new PlanningState(objects, facts));
	// cpp.setGoal(goal);
	// return cpp;
	// }

	/**
	 * @return
	 */
	private static MAPSIMAgent setupPlaymateProblem() {
		// try a grid world problem... this is from the mapsim code
		ObjectDeclaration objects[] = {
				new ObjectDeclaration("MrChips", "robot"),
				new ObjectDeclaration("MrChips", "planning_agent"),
				new ObjectDeclaration("black_start", "waypoint"),
				new ObjectDeclaration("blue_start", "waypoint"),
				new ObjectDeclaration("free_start", "waypoint"),
				new ObjectDeclaration("black_thing", "movable"),
				new ObjectDeclaration("blue_thing", "movable"),};

		Fact[] facts = {
				PlanningUtils.newFact("pos", new String[]{"black_thing"},
						"black_start"),
				PlanningUtils.newFact("pos", new String[]{"blue_thing"},
						"blue_start"),
				PlanningUtils.newFact("colour", new String[]{"blue_thing"},
						"blue"),
				PlanningUtils.newFact("colour", new String[]{"black_thing"},
						"black"),
				PlanningUtils.newFact("wp_left_of", new String[]{"free_start",
						"black_start"})

		};

		String goal = "(and\n  (left_of blue_thing black_thing)\n)";

		MAPSIMAgent cpp = MAPSIMAgentFactory.newPlayMateAgent(false);
		cpp.setInitialState(objects, facts);
		cpp.setGoal(goal);
		return cpp;
	}

	/**
	 * Include additional state that is always reused.
	 * 
	 * @param _additionalState
	 */
	public void addAdditionalState(PlanningStateLists _additionalState) {

		m_additionalState.extend(_additionalState);

		System.out.println("MAPSIMAgent.addAdditionalState(): "
				+ m_additionalState);
	}

}
