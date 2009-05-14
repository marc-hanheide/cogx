/**
 * 
 */
package planning.examples;

import java.util.ArrayList;
import java.util.Properties;

import planning.autogen.PlanningProcessRequest;
import planning.autogen.PlanningStateLists;
import planning.autogen.PlanningStatus;
import planning.util.PlannerServerFactory;
import planning.util.PlanningUtils;
import Planner.Fact;
import Planner.ObjectDeclaration;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.CASTUtils;

/**
 * Component that forms part of the planning example code. Writes an initial
 * state to working memory, then triggers a planning process on it.
 * 
 * @author nah
 */
public class Trigger extends PrivilegedManagedProcess
		implements
			WorkingMemoryChangeReceiver {

	private static final String PLAYMATE_AGENT_NAME = "MrChips";
	private static final String PLAYMATE_DOMAIN_NAME = "playmate";
	private static final String PLAYMATE_DOMAIN_FILE = "subarchitectures/planning.sa/src/python/mapsim/domains/playmate/mapl_files/domain.mapl";
	private Object m_trigger;

	/**
	 * @param _id
	 */
	public Trigger(String _id) {
		super(_id);
		m_objects = new ArrayList<ObjectDeclaration>();
		m_facts = new ArrayList<Fact>();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#runComponent()
	 */
	@Override
	protected void runComponent() {
		try {

			sleepProcess(2000);

			buildState();
			writeState();

//			 String goal = "(pos obj1 : mrchips)";
//			String goal = "(and (pos obj1 : dwp_0))";
//			 String goal = "(pos obj1 : wpa)";

//			 String goal = "(forall (?o0 - thing) (imply"
//			 + "(and (initially (colour ?o0 blue)))"
//			 + " (exists (?p0 - waypoint)"
//			 + " (exists (?o1 - thing ?p1 - waypoint)"
//			 + " (and (initially (colour ?o1 red)) (pos ?o0 ?p0) "
//			 + " (initially (pos ?o1 ?p1)) "
//			 + " (left ?p0 ?p1)" + ")))))";

			requestPlanningProcess();

			// sleepProcess(10000);
			// println("");
			// println("");
			// println("");
			// println("");
			// println("");
			// println("");
			//
			// goal = "(pos obj1 dwp_4)";
			//
			// requestPlanningProcess(goal);

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	private enum TriggerType {
		EXAMPLE, BINDING
	}
	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		if (_config.containsKey("--binding")) {
			m_trigger = TriggerType.BINDING;
		}
		else {
			m_trigger = TriggerType.EXAMPLE;
		}

	}

	private void requestPlanningProcess()
			throws SubarchitectureProcessException {

		if (m_trigger == TriggerType.EXAMPLE) {
			requestExamplePlanningProcess();
		}
		else if (m_trigger == TriggerType.BINDING) {
			requestBindingPlanningProcess();
		}
	}

	/**
	 * Request a planning process for the given goal using just the current
	 * subarchitecture and the playmate domain.
	 * 
	 * @param _goal
	 * @throws SubarchitectureProcessException
	 */
	private void requestExamplePlanningProcess()
			throws SubarchitectureProcessException {

		// String goal = "(pos obj1 : mrchips)";
		 String goal = "(and (pos obj1 : dwp_0))";	
//		String goal = "(and (wp_right_of dwp_11 wp9))";

		// String goal = "(forall (?o0 - thing) (imply"
		// + "(and (initially (colour ?o0 blue)))"
		// + " (exists (?p0 - waypoint)"
		// + " (exists (?o1 - thing ?p1 - waypoint)"
		// + " (and (initially (colour ?o1 red)) (pos ?o0 ?p0) "
		// + " (initially (pos ?o1 ?p1)) "
		// + " (left ?p0 ?p1)" + ")))))";

		PlanningProcessRequest ppr = new PlanningProcessRequest(goal,
				PLAYMATE_DOMAIN_NAME, PLAYMATE_DOMAIN_FILE,
				PLAYMATE_AGENT_NAME, new String[]{"action.sa"}, CASTUtils.workingMemoryPointer("","", PlanningProcessRequest.class),  true,
				PlanningStatus.PROPOSED, TriBool.triIndeterminate, new PlanningStateLists(new ObjectDeclaration[0],new Fact[0]));

		String id = newDataID();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.OVERWRITE), this);
		addToWorkingMemory(id, ppr);
	}

	/**
	 * Request a planning process for the given goal using just the current
	 * subarchitecture and the playmate domain.
	 * 
	 * @param _goal
	 * @throws SubarchitectureProcessException
	 */
	private void requestBindingPlanningProcess()
			throws SubarchitectureProcessException {

		sleepProcess(60000);
		// String goal = "(and (wp_left_of motmon8 motmon3))";
		String goal = "(and (pos motmon1 : motmon0))";
		// String goal = "(forall (?o0 - movable) (imply" + "(and "
		// + " (exists (?p0 - waypoint)" + " (and (pos ?o0 ?p0))" + "))";

		PlanningProcessRequest ppr = new PlanningProcessRequest(goal,
				PLAYMATE_DOMAIN_NAME, PLAYMATE_DOMAIN_FILE,
				PLAYMATE_AGENT_NAME, new String[]{"motivation.sa"}, CASTUtils.workingMemoryPointer("","", PlanningProcessRequest.class), true,
				PlanningStatus.PROPOSED, TriBool.triIndeterminate, new PlanningStateLists(new ObjectDeclaration[0],new Fact[0]));

		String id = newDataID();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.OVERWRITE), this);
		addToWorkingMemory(id, ppr);
	}

	private ArrayList<ObjectDeclaration> m_objects;

	private ArrayList<Fact> m_facts;

	private void buildState() {
		if (m_trigger == TriggerType.EXAMPLE) {
			buildExampleState();
		}

	}

	/**
	 * 
	 */
	private void buildExampleState() {
		// this is just a dummy at the moment
		m_objects.add(new ObjectDeclaration("obj1", "movable"));
		m_objects.add(new ObjectDeclaration("obj2", "movable"));
		m_objects.add(new ObjectDeclaration("obj3", "movable"));
		m_objects.add(new ObjectDeclaration("wpd", "waypoint"));
		m_objects.add(new ObjectDeclaration("wpa", "waypoint"));
		m_objects.add(new ObjectDeclaration("wp9", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_0", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_1", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_2", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_3", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_4", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_5", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_6", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_7", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_8", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_9", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_10", "waypoint"));
		m_objects.add(new ObjectDeclaration("dwp_11", "waypoint"));
		m_objects.add(new ObjectDeclaration(PLAYMATE_AGENT_NAME, "robot"));
		m_objects.add(new ObjectDeclaration(PLAYMATE_AGENT_NAME,
				"planning_agent"));

		m_facts.add(PlanningUtils.newFact("colour", new String[]{"obj1"},
				"blue"));
		m_facts.add(PlanningUtils.newFact("colour", new String[]{"obj2"},
				"blue"));
		m_facts.add(PlanningUtils
				.newFact("colour", new String[]{"obj3"}, "red"));
		m_facts.add(PlanningUtils.newFact("pos", new String[]{"obj1"}, "wpa"));
		m_facts.add(PlanningUtils.newFact("pos", new String[]{"obj2"}, "wp9"));
		m_facts.add(PlanningUtils.newFact("pos", new String[]{"obj3"}, "wpd"));

		m_facts.add(PlanningUtils.newFact("wp_left_of", new String[]{"dwp_0",
				"wpd"}));
		m_facts.add(PlanningUtils.newFact("wp_left_of", new String[]{"dwp_1",
				"wpd"}));
		m_facts.add(PlanningUtils.newFact("wp_left_of", new String[]{"dwp_2",
				"wpa"}));
		m_facts.add(PlanningUtils.newFact("wp_left_of", new String[]{"dwp_3",
				"wpa"}));
		m_facts.add(PlanningUtils.newFact("wp_left_of", new String[]{"dwp_4",
				"wp9"}));
		m_facts.add(PlanningUtils.newFact("wp_left_of", new String[]{"dwp_5",
				"wp9"}));

		m_facts.add(PlanningUtils.newFact("wp_right_of", new String[]{"dwp_6",
				"wpd"}));
		m_facts.add(PlanningUtils.newFact("wp_right_of", new String[]{"dwp_7",
				"wpd"}));
		m_facts.add(PlanningUtils.newFact("wp_right_of", new String[]{"dwp_8",
				"wpa"}));
		m_facts.add(PlanningUtils.newFact("wp_right_of", new String[]{"dwp_9",
				"wpa"}));
		m_facts.add(PlanningUtils.newFact("wp_right_of", new String[]{"dwp_10",
				"wp9"}));
		m_facts.add(PlanningUtils.newFact("wp_right_of", new String[]{"dwp_11",
				"wp9"}));
	}

	/**
	 * Adds the defined state to working memory.
	 * 
	 * @throws SubarchitectureProcessException
	 */
	private void writeState() throws SubarchitectureProcessException {

		for (ObjectDeclaration decl : m_objects) {
			log("writing object declaration: " + PlanningUtils.toString(decl));
			addToWorkingMemory(newDataID(), "action.sa", decl);
		}

		for (Fact fact : m_facts) {
			log("writing fact: " + PlanningUtils.toString(fact));
			addToWorkingMemory(newDataID(), "action.sa", fact);
		}

		flush();
	}

	/**
	 * Ensure all objects are written to working memory. Just a way of not
	 * calling flush on each add in writeState.
	 */
	private void flush() {
		if (m_inputToLocalWorkingMemory != null) {
			m_inputToLocalWorkingMemory.flush();
		}
		else {
			m_inputToRemoteWorkingMemory.flush();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {

	}

	public void workingMemoryChanged(WorkingMemoryChange _wmc) {

		try {
			PlanningProcessRequest ppr = (PlanningProcessRequest) getWorkingMemoryEntry(
					_wmc.m_address).getData();
			if (ppr.m_status == PlanningStatus.COMPLETE) {
				println("planning complete!: "
						+ CASTUtils.toString(ppr.m_succeeded));

				PlannerServerFactory.shutdown();

				sleepProcess(2000);

				// for testing purposes
				if (ppr.m_succeeded == TriBool.triTrue) {
					System.exit(CAST_TEST_PASS.value);
				}
				else {
					System.exit(CAST_TEST_FAIL.value);
				}
			}
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

}
