/**
 * 
 */
package planning.examples;

import java.util.Properties;

import Planner.Fact;
import Planner.ObjectDeclaration;
import Planner.PlannerServer;
import Planner.PlanningTask;

import planning.autogen.PlanningProcessRequest;
import planning.autogen.PlanningStateLists;
import planning.autogen.PlanningStatus;
import planning.util.PlannerServerFactory;
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
public class GoalTrigger extends PrivilegedManagedProcess
		implements
			WorkingMemoryChangeReceiver {

	private PlannerServer m_planner;

	private String m_task_fn;
	private String m_domain_fn;
	private String m_agent_name;
	private String m_domain_name;
	private String m_goal;

	/**
	 * @param _id
	 */
	public GoalTrigger(String _id) {
		super(_id);
		
		m_planner = PlannerServerFactory.getPlannerServer();

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#runComponent()
	 */
	@Override
	protected void runComponent() {
		try {

			sleepProcess(2500);


			loadTask();
			requestPlanningProcess(m_goal);

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

	/**
	 * 
	 */
	private void loadTask() {
		if (m_task_fn == null || m_domain_fn == null || m_agent_name == null) {
			return;
		}
		
		PlanningTask task = m_planner.load_mapl_task(m_task_fn, m_domain_fn, m_agent_name);
		m_goal = task.goals[0];
		m_domain_name = task.domain_name;
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
				m_domain_name, m_domain_fn, m_agent_name,
				new String[]{"action.sa"}, CASTUtils.workingMemoryPointer("","", PlanningProcessRequest.class), true,
				PlanningStatus.PROPOSED, TriBool.triIndeterminate, new PlanningStateLists(new ObjectDeclaration[0],new Fact[0]));
		log("setting goal: " + _goal);

		String id = newDataID();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.OVERWRITE), this);
		addToWorkingMemory(id, ppr);
	}

	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		m_task_fn = _config.getProperty("--task");
		m_domain_fn = _config.getProperty("--domain");
		m_agent_name = _config.getProperty("--agent");
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
				
				//for testing purposes
				if(ppr.m_succeeded == TriBool.triTrue) {
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
