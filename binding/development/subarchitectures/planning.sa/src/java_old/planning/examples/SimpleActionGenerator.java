/**
 * 
 */
package planning.examples;

import planning.autogen.PlanningData.Action;
import planning.autogen.PlanningData.ActionRegistration;
import planning.autogen.PlanningData.PlanningStatus;
import planning.ontology.PlanningOntology;
import planning.ontology.PlanningOntologyFactory;
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
public class SimpleActionGenerator extends ManagedProcess implements
		WorkingMemoryChangeReceiver {

	/**
	 * @param _id
	 */
	public SimpleActionGenerator(String _id) {
		super(_id);
		setOntology(PlanningOntologyFactory.getOntology());
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
			addChangeFilter(PlanningOntology.ACTION_TYPE,
					WorkingMemoryOperation.ADD, true, this);
		} catch (SubarchitectureProcessException e) {
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
		// TODO Auto-generated method stub

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
		// TODO Auto-generated method stub

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
			if (Math.random() > 0.5) {
				performAction(action);
			} else {
				log("not performing action");
			}

			// fix these bits for the time being
			action.m_status = PlanningStatus.COMPLETE;
			action.m_succeeded = TriBool.triTrue;
			overwriteWorkingMemory(_wmc.m_address.m_id,
					PlanningOntology.ACTION_TYPE, action);
		} catch (SubarchitectureProcessException e) {
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
		String step = (String) getWorkingMemoryEntry(
				_action.m_action.m_address.m_id).getData();

		log("performing step: " + step);
		String[] addList = getAdditions(step);
		String[] delList = getDeletions(step);

		for (String addition : addList) {
			log("adding fact: " + addition);
			addToWorkingMemory(newDataID(), PlanningOntology.FACT_TYPE,
					addition);
		}

		CASTData<?>[] facts = getWorkingMemoryEntries(
				PlanningOntology.FACT_TYPE, 0);

		// this is ugly
		for (CASTData<?> factData : facts) {
			String fact = (String) factData.getData();
			for (String deletion : delList) {
				// log("comparing: " + deletion + " " + fact);
				if (fact.toLowerCase().equals(deletion)) {
					log("deleting fact: " + deletion + " " + factData.getId());
					deleteFromWorkingMemory(factData.getId());
				}
			}
		}
	}

	/**
	 * @param _step
	 * @return
	 */
	private String[] getDeletions(String _step) {
		String[] bits = _step.split(" ");

		if (bits[0].equals("pick")) {
			return new String[] { "pos " + bits[2] + " " + bits[3] };
		} else if (bits[0].equals("place")) {
			return new String[] { "pos " + bits[2] + " mr_chips" };
		} else if (bits[0].equals("move")) {
			// log("return ing deletions for move");
			return new String[] { "pos " + bits[2] + " " + bits[4] };
		} else {
			// log("return ing NO deletions for move");
			return new String[] {};
		}
	}

	/**
	 * @param _step
	 * @return
	 */
	private String[] getAdditions(String _step) {
		String[] bits = _step.split(" ");

		if (bits[0].equals("pick")) {
			return new String[] { "pos " + bits[2] + " mr_chips" };
		} else if (bits[0].equals("place") || bits[0].equals("move")) {
			return new String[] { "pos " + bits[2] + " " + bits[3] };
		} else if (bits[0].equals("move")) {
			return new String[] { "pos " + bits[2] + " " + bits[3] };
		}

		else {
			return new String[] {};
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

			ActionRegistration pick = new ActionRegistration(
					getProcessIdentifier(), getSubarchitectureID(), "pick");
			addToWorkingMemory(newDataID(),
					PlanningOntology.ACTION_REGISTRATION_TYPE, pick);

			ActionRegistration place = new ActionRegistration(
					getProcessIdentifier(), getSubarchitectureID(), "place");
			addToWorkingMemory(newDataID(),
					PlanningOntology.ACTION_REGISTRATION_TYPE, place);

			// new action type!!! ow

			ActionRegistration move = new ActionRegistration(
					getProcessIdentifier(), getSubarchitectureID(), "move");
			addToWorkingMemory(newDataID(),
					PlanningOntology.ACTION_REGISTRATION_TYPE, move);

		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.exit(1);
		}

	}

}
