/**
 * 
 */
package planning.examples;

import java.util.ArrayList;

import planning.autogen.Planner.ObjectDeclaration;
import planning.autogen.PlanningData.PlanningProcessRequest;
import planning.autogen.PlanningData.PlanningStatus;
import planning.ontology.PlanningOntology;
import planning.ontology.PlanningOntologyFactory;
import planning.util.PlanningUtils;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;

/**
 * Component that forms part of the planning example code. Writes an initial
 * state to working memory, then triggers a planning process on it.
 * 
 * @author nah
 */
public class Trigger extends PrivilegedManagedProcess {

	private final static String PLAYMATE_DOMAIN = "./subarchitectures/planning.sa/config/domains/playmate-domain.mapl";

	/**
	 * @param _id
	 */
	public Trigger(String _id) {
		super(_id);
		setOntology(PlanningOntologyFactory.getOntology());
		m_objects = new ArrayList<ObjectDeclaration>();
		m_facts = new ArrayList<String>();
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

			// String goal = "(pos obj1 mr_chips)";
			 String goal = "(pos obj1 dwp_0)";
//			String goal = "(pos obj1 wpa)";

			// String goal = "(forall (?o0 - thing) (imply"
			// + "(and (initially (colour ?o0 blue)))"
			// + " (exists (?p0 - waypoint)"
			// + " (exists (?o1 - thing ?p1 - waypoint)"
			// + " (and (initially (colour ?o1 red)) (pos ?o0 ?p0) "
			// + " (initially (pos ?o1 ?p1)) "
			// + " (left ?p0 ?p1)" + ")))))";

			requestPlanningProcess(goal);

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

		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Request a planning process for the given goal using just the current
	 * subarchitecture and the playmate domain.
	 * 
	 * @param _goal
	 * @throws SubarchitectureProcessException
	 */
	private void requestPlanningProcess(String _goal)
			throws SubarchitectureProcessException {
		PlanningProcessRequest ppr = new PlanningProcessRequest(_goal,
				PLAYMATE_DOMAIN, new String[] { "action.sa" }, true,
				PlanningStatus.PROPOSED, TriBool.triIndeterminate);

		addToWorkingMemory(newDataID(),
				PlanningOntology.PLANNING_PROCESS_REQUEST_TYPE, ppr);
	}

	private ArrayList<ObjectDeclaration> m_objects;

	private ArrayList<String> m_facts;

	/**
	 * 
	 */
	private void buildState() {
		// this is just a dummy at the moment
		m_objects.add(new ObjectDeclaration("obj1", "thing"));
		m_objects.add(new ObjectDeclaration("obj2", "thing"));
		m_objects.add(new ObjectDeclaration("obj3", "thing"));
		m_objects.add(new ObjectDeclaration("wpD", "waypoint"));
		m_objects.add(new ObjectDeclaration("wpA", "waypoint"));
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

		m_facts.add("colour obj1 blue");
		m_facts.add("colour obj2 blue");
		m_facts.add("colour obj3 red");
		m_facts.add("pos obj1 wpA");
		m_facts.add("pos obj2 wp9");
		m_facts.add("pos obj3 wpD");

		m_facts.add("left dwp_0 wpD");
		m_facts.add("left dwp_1 wpD");
		m_facts.add("left dwp_2 wpA");
		m_facts.add("left dwp_3 wpA");
		m_facts.add("left dwp_4 wp9");
		m_facts.add("left dwp_5 wp9");
		m_facts.add("right dwp_6 wpD");
		m_facts.add("right dwp_7 wpD");
		m_facts.add("right dwp_8 wpA");
		m_facts.add("right dwp_9 wpA");
		m_facts.add("right dwp_10 wp9");
		m_facts.add("right dwp_11 wp9");
	}

	/**
	 * Adds the defined state to working memory.
	 * 
	 * @throws SubarchitectureProcessException
	 */
	private void writeState() throws SubarchitectureProcessException {

		for (ObjectDeclaration decl : m_objects) {
			log("writing object declaration: " + PlanningUtils.toString(decl));
			addToWorkingMemory(newDataID(), "action.sa",
					PlanningOntology.OBJECT_DECLARATION_TYPE, decl);
		}

		for (String fact : m_facts) {
			log("writing fact: (" + fact + ")");
			addToWorkingMemory(newDataID(), "action.sa",
					PlanningOntology.FACT_TYPE, fact);
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
		} else {
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

}
