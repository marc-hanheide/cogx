/**
 * 
 */
package planning.examples;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Properties;

import planning.util.PlannerServerFactory;
import planning.util.PlanningUtils;
import Planner.Fact;
import Planner.ObjectDeclaration;
import Planner.PlannerServer;
import Planner.PlanningTask;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;

/**
 * Component that forms part of the planning example code. Writes an initial
 * state to working memory, then triggers a planning process on it.
 * 
 * @author nah
 */
public class StateTrigger extends PrivilegedManagedProcess {
	
	private PlanningTask m_task = null;
	private PlannerServer m_planner;

	private String m_task_fn;
	private String m_domain_fn;
	private String m_agent_name;
	
	/**
	 * @param _id
	 */
	public StateTrigger(String _id) {
		super(_id);
		m_planner = PlannerServerFactory.getPlannerServer();

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

			loadState();
			writeState();

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}


	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		m_task_fn = _config.getProperty("--task");
		m_domain_fn = _config.getProperty("--domain");
		m_agent_name = _config.getProperty("--agent");
	}

	private ArrayList<ObjectDeclaration> m_objects;

	private ArrayList<Fact> m_facts;

	/**
	 * 
	 */
	private void loadState() {
		if (m_task_fn == null || m_domain_fn == null || m_agent_name == null) {
			return;
		}
		
		m_task = m_planner.load_mapl_task(m_task_fn, m_domain_fn, m_agent_name);
		m_objects.addAll(Arrays.asList(m_task.objects));
		m_facts.addAll(Arrays.asList(m_task.facts));

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

}
