/**
 * 
 */
package planning.examples;

import java.util.Properties;

import Planner.Command;
import Planner.Fact;
import planning.autogen.Action;
import planning.autogen.ActionRegistration;
import planning.autogen.PlanningStatus;
import planning.util.PlanningUtils;
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
public class SimpleActionGenerator extends ManagedProcess
		implements
			WorkingMemoryChangeReceiver {

	private double m_failureChance;

	/**
	 * @param _id
	 */
	public SimpleActionGenerator(String _id) {
		super(_id);
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
		
		log("chance of failure: " + m_failureChance);
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
		Fact[] addList = getAdditions(step);
		Fact[] delList = getDeletions(step);

		for (Fact addition : addList) {
			log("adding fact: " + PlanningUtils.toString(addition));
			addToWorkingMemory(newDataID(), addition);
		}

		CASTData<Fact>[] facts = getWorkingMemoryEntries(Fact.class);

		// this is ugly
		for (CASTData<Fact> factData : facts) {
			Fact fact = factData.getData();
			for (Fact deletion : delList) {
				// log("comparing: " + PlanningUtils.toString(deletion) + " " +
				// PlanningUtils.toString(fact));
				if (PlanningUtils.equals(fact, deletion)) {
					log("deleting fact: " + PlanningUtils.toString(deletion)
							+ " " + factData.getID());
					deleteFromWorkingMemory(factData.getID());
				}
			}
		}
	}

	/**
	 * @param _step
	 * @return
	 */
	private Fact[] getDeletions(Command _step) {

		if (_step.mapl_action.name.equals("pick_up")) {
			return new Fact[]{PlanningUtils.newFact("pos",
					new String[]{_step.mapl_action.args[0]},
					_step.mapl_action.args[1])};
		}
		else if (_step.mapl_action.name.equals("place")) {
			return new Fact[]{PlanningUtils.newFact("pos",
					new String[]{_step.mapl_action.args[0]}, "mrchips")};
		}
		else if (_step.mapl_action.name.equals("move")) {
			println("return ing deletions for move... probably doesn't working properly!");
			return new Fact[]{PlanningUtils.newFact("pos", new String[]{
					_step.mapl_action.args[1]}, _step.mapl_action.args[3])};
		}
		else {
			// log("return ing NO deletions for move");
			return new Fact[]{};
		}
	}

	/**
	 * @param _step
	 * @return
	 */
	private Fact[] getAdditions(Command _step) {

		if (_step.mapl_action.name.equals("pick_up")) {
			return new Fact[]{PlanningUtils.newFact("pos",
					new String[]{_step.mapl_action.args[0]}, "mrchips")};
		}
		else if (_step.mapl_action.name.equals("place")
				|| _step.mapl_action.equals("move")) {
			return new Fact[]{PlanningUtils.newFact("pos",
					new String[]{_step.mapl_action.args[0]},
					_step.mapl_action.args[1])};
		}

		else {
			return new Fact[]{};
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

			ActionRegistration pick_up = new ActionRegistration(
					getProcessIdentifier(), getSubarchitectureID(), "pick_up");
			addToWorkingMemory(newDataID(), pick_up);

			ActionRegistration place = new ActionRegistration(
					getProcessIdentifier(), getSubarchitectureID(), "place");
			addToWorkingMemory(newDataID(), place);

			// new action type!!! ow

			ActionRegistration move = new ActionRegistration(
					getProcessIdentifier(), getSubarchitectureID(), "move");
			addToWorkingMemory(newDataID(), move);

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
		}

	}

}
