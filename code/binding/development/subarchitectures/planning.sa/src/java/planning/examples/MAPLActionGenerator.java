/**
 * 
 */
package planning.examples;

import java.util.Properties;

import planning.autogen.Action;
import planning.autogen.ActionRegistration;
import planning.autogen.PlanningStatus;
import planning.util.PlannerServerFactory;
import planning.util.PlanningUtils;
import Planner.Command;
import Planner.Fact;
import Planner.PlannerServer;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;

/**
 * @author nah
 */
public class MAPLActionGenerator extends ManagedProcess
		implements
			WorkingMemoryChangeReceiver {

	private static final String[] actions = { "pick_up", "place", "move", "grasp", "drop" };
	
	private PlannerServer m_planner;

	private double m_failureChance;

	/**
	 * @param _id
	 */
	public MAPLActionGenerator(String _id) {
		super(_id);
		
		m_planner = PlannerServerFactory.getPlannerServer();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();

		try {
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					Action.class, WorkingMemoryOperation.ADD), this);
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
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

	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		String property = _config.getProperty("--failure");
		if (property != null) {
			m_failureChance = Double.parseDouble(property);
		}
		else {
			m_failureChance = 0.5;
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryChangeReceiver#workingMemoryChanged(cast.cdl.WorkingMemoryChange)
	 */
	public void workingMemoryChanged(WorkingMemoryChange _wmc) {

		// to start with, do nothing
		try {
			// sleepProcess(200); //pretend we take some time
			Action action = (Action) getWorkingMemoryEntry(_wmc.m_address)
					.getData();

			// randomly do the action
			if (Math.random() >= m_failureChance) {
				performAction(action);
			}
			else {
				log("not performing action");
			}

			// fix these bits for the time being
			action.m_status = PlanningStatus.COMPLETE;
			action.m_succeeded = TriBool.triTrue;
			overwriteWorkingMemory(_wmc.m_address.m_id, action);
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
		}

	}

	/**
	 * @param _action
	 * @throws SubarchitectureProcessException
	 */
	private void performAction(Action _action)
			throws SubarchitectureProcessException {
		Command step = (Command) getWorkingMemoryEntry(
				_action.m_action.m_address).getData();

		log("performing step: " + PlanningUtils.toString(step));
		Fact[] changes = m_planner.expected_changes(step.task_id, step);

		for (Fact fact : changes) {
			log("adding fact: " + PlanningUtils.toString(fact));
			addToWorkingMemory(newDataID(), fact);
		}

		CASTData<Fact>[] facts = getWorkingMemoryEntries(Fact.class);

		// Remove facts that have been overwritten
		for (CASTData<Fact> factData : facts) {
			Fact fact = factData.getData();
			for (Fact change : changes) {
				if (PlanningUtils.sameFactDifferentValue(fact, change)) {
					log("deleting fact: " + PlanningUtils.toString(fact)
							+ " " + factData.getID());
					deleteFromWorkingMemory(factData.getID());
				}
			}
		}
	}


	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#runComponent()
	 */
	@Override
	protected void runComponent() {
		try {
			// register actions
			for (String action : actions) {
				ActionRegistration action_reg = new ActionRegistration(
						getProcessIdentifier(), getSubarchitectureID(), action);
				addToWorkingMemory(newDataID(), action_reg);
			}
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
		}

	}

}
