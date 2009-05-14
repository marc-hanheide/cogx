/**
 * 
 */
package planning.components.abstr;

import java.util.Hashtable;

import planning.autogen.PlanningStateLists;
import planning.autogen.PlanningStateRequest;
import planning.util.PlanningDataTranslator;
import planning.util.TemporaryPlanningState;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TaskOutcome;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;

/**
 * Superclass for components that can generate logical descriptions of the the
 * current state from the contents of working memory. <br/> When an instance of
 * PlanningOntology.PLANNING_STATE_REQUEST_TYPE is added to this component's
 * sa-local working memory it reads in this object and generates a state to
 * fulfil the request.
 * 
 * @author nah
 */
public abstract class PlanningStateGenerator extends PrivilegedManagedProcess {

	private Hashtable<Class<?>, String> m_subarchitectures;

	private Hashtable<Class<?>, PlanningDataTranslator> m_translators;

	private Hashtable<String, WorkingMemoryAddress> m_taskMap;

	/**
	 * @param _id
	 */
	public PlanningStateGenerator(String _id) {
		super(_id);
		m_translators = new Hashtable<Class<?>, PlanningDataTranslator>();
		m_subarchitectures = new Hashtable<Class<?>, String>();
		m_taskMap = new Hashtable<String, WorkingMemoryAddress>();
	}

	/**
	 * Adds the contents of _substate to _state.
	 * 
	 * @param _state
	 * @param _substate
	 */
	private void extendPlanningState(TemporaryPlanningState _state,
			TemporaryPlanningState _substate) {
		// for the time being, let's just be dumb about it
		_state.m_factList.addAll(_substate.m_factList);
		_state.m_objectList.addAll(_substate.m_objectList);
	}

	protected PlanningStateLists generateState()
			throws SubarchitectureProcessException {

		TemporaryPlanningState state = new TemporaryPlanningState();

		for (Class<?> type : m_translators.keySet()) {

			String subarchitecture = m_subarchitectures.get(type);
			CASTData<?>[] wmel = getWorkingMemoryEntries(subarchitecture, type,
					0);

			PlanningDataTranslator translator = m_translators.get(type);

			TemporaryPlanningState substate = translator.toPlanningState(wmel);
			if (substate != null) {
				extendPlanningState(state, substate);
			}

		}

		return state.toPlanningState();
	}

	/**
	 * Register an object to be used to translate a particular state entry into
	 * a planning state.
	 * 
	 * @param <T>
	 * @param _ontologicalType
	 * @param _translator
	 */
	protected <T> void registerPlanStateMapping(Class<T> _type,
			PlanningDataTranslator _translator) {
		registerPlanStateMapping(_type, m_subarchitectureID, _translator);
	}

	/**
	 * Register an object to be used to translate a particular state entry into
	 * a planning state.
	 * 
	 * @param <T>
	 * @param _ontologicalType
	 * @param _translator
	 */
	protected <T> void registerPlanStateMapping(Class<T> _type,
			String _subarch, PlanningDataTranslator _translator) {
		m_translators.put(_type, _translator);
		m_subarchitectures.put(_type, _subarch);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {
		WorkingMemoryAddress requestAddr = m_taskMap.get(_taskID);
		if (requestAddr == null) {
			throw new RuntimeException("unknown task: " + _taskID);
		}
		PlanningStateRequest psr = null;
		try {
			psr = (PlanningStateRequest) getWorkingMemoryEntry(requestAddr)
					.getData();

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
		}

		// the state should be empty
		if (psr.m_state.m_facts.length != 0
				|| psr.m_state.m_objects.length != 0) {
			log("state already written by someone else");
			try {
				taskComplete(_taskID, TaskOutcome.PROCESSING_COMPLETE_FAILURE);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				System.exit(1);
			}
			return;
		}

		try {
			// generate state via subclass
			psr.m_state = generateState();
			// overwrite the request
			overwriteWorkingMemory(requestAddr.m_id,
					requestAddr.m_subarchitecture, psr);
			// signal complete
			taskComplete(_taskID, TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
		}
		catch (SubarchitectureProcessException e) {
			// if something goes wrong
			try {
				taskComplete(_taskID, TaskOutcome.PROCESSING_COMPLETE_FAILURE);
			}
			catch (SubarchitectureProcessException e1) {
				// now we're really screwed
				e1.printStackTrace();
				System.exit(1);
			}
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
		m_taskMap.remove(_taskID);
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

			// listen for state requests that only occur locally
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					PlanningStateRequest.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								String taskID = newTaskID();
								m_taskMap.put(taskID, _wmc.m_address);
								proposeInformationProcessingTask(taskID,
										"fulfil-state-request-"
												+ _wmc.m_address.m_id);
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
								System.exit(1);
							}
						}
					});
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
		}

	}

}
