/**
 * 
 */
package planning.components;

import java.util.Hashtable;
import java.util.Properties;
import java.util.Vector;

import planning.autogen.Action;
import planning.autogen.ActionRegistration;
import planning.autogen.PlanningProcessRequest;
import planning.autogen.PlanningStateLists;
import planning.autogen.PlanningStateRequest;
import planning.autogen.PlanningStatus;
import planning.util.MAPSIMAgent;
import planning.util.MAPSIMAgentFactory;
import planning.util.PlannerServerFactory;
import planning.util.PlanningException;
import planning.util.PlanningSubarchitectureException;
import planning.util.PlanningUtils;
import planning.util.TemporaryPlanningState;
import Planner.Command;
import Planner.Fact;
import Planner.Failure;
import Planner.ObjectDeclaration;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TaskOutcome;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;

/**
 * Process that does most of the continual planning work.
 * 
 * @author nah
 */
public class ContinualPlanningComponent extends PrivilegedManagedProcess {

	private class ActionReceiver implements WorkingMemoryChangeReceiver {

		/*
		 * (non-Javadoc)
		 * 
		 * @see cast.architecture.abstr.WorkingMemoryChangeReceiver#workingMemoryChanged(cast.cdl.WorkingMemoryChange)
		 */
		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			assert _wmc.m_type.equals(CASTUtils.typeName(Action.class)) : "Only setup to handle one type";
			try {
				log("CPC: seen WM change");
				Action action = (Action) getWorkingMemoryEntry(_wmc.m_address)
						.getData();

				if (action.m_status == PlanningStatus.COMPLETE) {
					// ok, we're done here
					removeChangeFilter(this);
					// delete general action
					deleteFromWorkingMemory(_wmc.m_address.m_id,
							_wmc.m_address.m_subarchitecture);
					// delete action specification
					deleteFromWorkingMemory(action.m_action.m_address.m_id,
							action.m_action.m_address.m_subarchitecture);
					actionComplete(action);
				}

			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}

	}

	private class StateRequestReceiver implements WorkingMemoryChangeReceiver {
		/*
		 * (non-Javadoc)
		 * 
		 * @see cast.architecture.abstr.WorkingMemoryChangeReceiver#workingMemoryChanged(cast.cdl.WorkingMemoryChange)
		 */
		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			assert _wmc.m_type.equals(CASTUtils
					.typeName(PlanningStateRequest.class)) : "Only setup to handle one type";
			try {
				println("received change");
				// ok, we're done here
				removeChangeFilter(this);
				m_requestFilters.remove(this);
				requestAnswered(_wmc.m_address);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}

	}

	/**
	 * 
	 */
	private static final String BUILD_INITIAL_STATE = "start-planning-process";

	private static final String CREATE_PLAN = "create-plan";

	private static final String EXECUTE_PLAN = "execute-plan";

	private Hashtable<String, WorkingMemoryChange> m_taskMap;

	private MAPSIMAgent m_cpp;

	private TemporaryPlanningState m_state;

	private Vector<StateRequestReceiver> m_requestFilters;

	private String m_currentTaskID;

	private WorkingMemoryAddress m_pprAddr;

	private Object m_runComponentLock = new Object();

	private Action m_lastCompletedAction;

	private int m_replanLimit = 1;

	private Hashtable<String, String> m_actionSubachMap;

	private WorkingMemoryPointer m_pprCause;

	/**
	 * @param _id
	 */
	public ContinualPlanningComponent(String _id) {
		super(_id);
		m_taskMap = new Hashtable<String, WorkingMemoryChange>();
		m_requestFilters = new Vector<StateRequestReceiver>();
		m_actionSubachMap = new Hashtable<String, String>();
		m_lastCompletedAction = null;
	}

	/**
	 * @param _action
	 */
	private void actionComplete(Action _action) {
		// HACK sleep to allow vision time to process after arm has finished
		// log("CPC: actionComplete...");
		// int sleeps = 6;
		// for (int i = 0; i < sleeps; i++) {
		// log("sleep: " + (sleeps - i));
		// sleepProcess(1000);
		// }

		m_lastCompletedAction = _action;
		notifyRunComponent();
	}

	/**
	 * @param __taskid
	 * @throws SubarchitectureProcessException
	 */
	private void buildStateTask() throws SubarchitectureProcessException {
		WorkingMemoryChange wmc = m_taskMap.remove(m_currentTaskID);
		if (wmc == null) {
			println("received unknown taskid: " + m_currentTaskID);
			return;
		}
		log("CPC: buildStateTask...");
		assert wmc.m_type.equals(CASTUtils
				.typeName(PlanningProcessRequest.class)) : "Can only handle PlanningProcessRequest objects";

		PlanningProcessRequest ppr = (PlanningProcessRequest) getWorkingMemoryEntry(
				wmc.m_address).getData();

		if (m_cpp != null) {
			throw new PlanningSubarchitectureException(
					"A continual planning process is already in process. Currently cannot handle more than one.");
		}

		// zero the state
		m_state = null;

		// and store the location of the request so we can update it as
		// we go
		m_pprAddr = wmc.m_address;

		m_pprCause = ppr.m_cause;
		
		setRequestStatus(PlanningStatus.PREPARING);
		// create the object that does the planning magic
		log("CPC: buildStateTask -- creating new CPP...");

		// m_cpp = new ContinualPlanningProcess(ppr.m_domainFile,
		// m_bDebugOutput); // only
		m_cpp = MAPSIMAgentFactory.newMAPSIMAgent(ppr.m_domainName,
				ppr.m_domainFile, ppr.m_agent, false);

		m_cpp.addAdditionalState(ppr.m_additionalState);

		// when
		// we
		// really
		// need
		// it

		// give it our goal
		log("CPC: buildStateTask -- setting goal..." + ppr.m_maplGoal);
		m_cpp.setGoal(ppr.m_maplGoal);

		// start requesting planning states
		requestStates(ppr, wmc.m_address);

	}

	/**
	 * @throws Failure
	 * @throws SubarchitectureProcessException
	 */
	private void createPlanTask() throws Failure,
			SubarchitectureProcessException {

		// some sanity checks
		assert m_cpp != null && m_state != null : "processing should have been started by now";

		if (m_state.m_objectList.isEmpty()) {
			throw new PlanningSubarchitectureException(
					"Initial state not properly defined");
		}

		setRequestStatus(PlanningStatus.PLANNING);

		// set the intial state on the planner
		PlanningStateLists lists = m_state.toPlanningState();
		m_cpp.setInitialState(lists.m_objects, lists.m_facts);

		// now we can throw away the previous state
		m_state = null;

		// SimplePOPlan plan = m_cpp.getPlan();
		try {
			m_cpp.startPlanning();
		}
		catch (PlanningException e) {
			println(e);
			executionFailed();
			return;
		}

		if (m_cpp.goalAcheivable()) {
			log("goal is achievable");
			currentTaskSucceeded();
			// now go for execution
			proposeTask(EXECUTE_PLAN);
		}
		else if (m_cpp.goalAchieved()) {
			log("goal achieved in initial state");
			executionSucceeded();
		}
		else {
			println("goal is not achievable from initial state");
			executionFailed();
		}

	}

	/**
	 * Signal that the current task has failed.
	 */
	private void currentTaskFailed() {
		assert m_currentTaskID != null;
		// if any tasks fail, assume whole process has failed
		try {
			taskComplete(m_currentTaskID,
					TaskOutcome.PROCESSING_COMPLETE_FAILURE);
			m_currentTaskID = null;
		}
		catch (SubarchitectureProcessException e1) {
			e1.printStackTrace();
			System.exit(1);
		}
	}

	/**
	 * Signal that the current task has succeeded.
	 */
	private void currentTaskSucceeded() {
		assert m_currentTaskID != null;
		// if any tasks fail, assume whole process has failed
		try {
			taskComplete(m_currentTaskID,
					TaskOutcome.PROCESSING_COMPLETE_FAILURE);
			m_currentTaskID = null;
		}
		catch (SubarchitectureProcessException e1) {
			e1.printStackTrace();
			System.exit(1);
		}
	}

	/**
	 * This is where things get a bit complicated. What needs to happen is the
	 * following... 1. build state 2. check executability of next action 3. if
	 * pass, execute action 4. check action feedback 5. if pass, loop 6. if no
	 * more actions 7. build state 8. check for goal
	 * 
	 * @param _taskID
	 * @throws SubarchitectureProcessException
	 */
	private void executePlanTask() throws SubarchitectureProcessException {
		log("CPC: executing planned task");
		setRequestStatus(PlanningStatus.EXECUTING);

		notifyRunComponent();
	}

	/**
	 * @throws SubarchitectureProcessException
	 */
	private void executionFailed() throws SubarchitectureProcessException {
		log("CPC: execution failed");

		setRequestStatus(PlanningStatus.COMPLETE, TriBool.triFalse);
		currentTaskFailed();
		// HACK: fix this up
		m_currentTaskID = null;

		reset();
	}

	/**
	 * @throws SubarchitectureProcessException
	 */
	private void executionSucceeded() throws SubarchitectureProcessException {
		log("execution suceeded");

		setRequestStatus(PlanningStatus.COMPLETE, TriBool.triTrue);
		currentTaskSucceeded();
		// HACK: fix this up
		m_currentTaskID = null;

		reset();

	}

	/**
	 * Reset member variables used by planning process
	 */
	private void reset() {
		log("CPC: planning process reset");
		// remove the planning process
		m_cpp = null;

		// zero the state
		m_state = null;

		// and the request
		m_pprAddr = null;
		// and its cause
		m_pprCause = null;
		
	}

	/**
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	private PlanningProcessRequest getCurrentRequest()
			throws SubarchitectureProcessException {
		assert m_pprAddr != null;
		return (PlanningProcessRequest) getWorkingMemoryEntry(m_pprAddr)
				.getData();
	}

	/**
	 * Just a helper to ensure everything is setup correctly.
	 * 
	 * @return An empty state request
	 */
	private PlanningStateRequest newPlanningStateRequest() {
		return newPlanningStateRequest(new WorkingMemoryAddress("", ""));
	}

	/**
	 * Just a helper to ensure everything is setup correctly.
	 * 
	 * @return An empty state request
	 */
	private PlanningStateRequest newPlanningStateRequest(
			WorkingMemoryAddress _addr) {
		PlanningStateRequest psr = new PlanningStateRequest(
				new PlanningStateLists(new ObjectDeclaration[0], new Fact[0]),
				_addr);
		return psr;
	}

	/**
	 * 
	 */
	private void notifyRunComponent() {
		// wake up run loop, as it's play time
		synchronized (m_runComponentLock) {
			m_runComponentLock.notifyAll();
			// log("CPC: notified");
		}
	}

	/**
	 * @param _task
	 * @throws SubarchitectureProcessException
	 */
	private String proposeTask(String _task) {
		// then ask if we can plan
		String taskID = _task + ":" + newTaskID();
		try {
			// log("CPC: proposing information processing task: " + taskID);
			proposeInformationProcessingTask(taskID, _task);
		}
		catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.exit(1);
		}
		return taskID;
	}

	// /**
	// * Replan for current goal using recently generated state
	// *
	// * @throws SubarchitectureProcessException
	// * @throws Failure
	// */
	// private void replan() throws SubarchitectureProcessException, Failure {
	//
	// // some sanity checks
	// assert m_cpp != null && m_state != null : "processing should have been
	// started by now";
	//
	// if (m_state.m_factList.isEmpty() || m_state.m_objectList.isEmpty()) {
	// throw new PlanningSubarchitectureException(
	// "Initial state not properly defined");
	// }
	//
	// // probably don't bother unless really necessary
	// // setRequestStatus(PlanningStatus.REPLANNING);
	//
	// // get a new plan using the last state update
	// log("CPC: getting new plan by replanning");
	// SimplePOPlan plan = m_cpp.getPlan();
	// log("CPC: found replan: " + Arrays.toString(plan.nodes));
	// }

	/**
	 * Receive an answer to a state request
	 * 
	 * @param _address
	 * @throws SubarchitectureProcessException
	 */
	private void requestAnswered(WorkingMemoryAddress _wma)
			throws SubarchitectureProcessException {

		// get the request
		PlanningStateRequest psr = (PlanningStateRequest) getWorkingMemoryEntry(
				_wma).getData();

		// lazy creation
		if (m_state == null) {
			m_state = new TemporaryPlanningState();
		}
		else {
			// log("state already contains: " + m_state);
		}

		// log("CPC: state request answered: extending state...");

		// extended state with collection
		m_state.extend(psr.m_state);

		// println("CPC: state size after extension: " +
		// m_state.m_factList.size()
		// + " " + m_state.m_objectList.size());

		// log("CPC: extended state contains: ");
		// for( String s : m_state.m_factList ) {
		// log("state fact: " + s );
		// }
		// for( ObjectDeclaration d : m_state.m_objectList ) {
		// log("object decl: " + d.name + " " + d.type);
		// }

		// if that was the last request to be fulfilled
		if (m_requestFilters.isEmpty()) {
			// if we were building an initial state
			if (m_currentTaskID.startsWith(BUILD_INITIAL_STATE)) {
				currentTaskSucceeded();
				// then ask if we can plan
				proposeTask(CREATE_PLAN);
			}
			else {
				notifyRunComponent();
			}
		}
	}

	/**
	 * Execution specialised method for build current state.
	 * 
	 * @throws SubarchitectureProcessException
	 */
	private void requestCurrentState() throws SubarchitectureProcessException {
		PlanningProcessRequest ppr = getCurrentRequest();
		requestStates(ppr, m_pprAddr);
	}

	/**
	 * Request planning states from the subarchitectures involved in the
	 * planning problem.
	 * 
	 * @param _ppr
	 * @param _addr
	 * @throws SubarchitectureProcessException
	 */
	private void requestStates(PlanningProcessRequest _ppr,
			WorkingMemoryAddress _addr) throws SubarchitectureProcessException {

		// zero current state!
		m_state = null;

		String[] subarchs = _ppr.m_contributors;
		for (String sa : subarchs) {

			String id = newDataID();
			PlanningStateRequest psr = newPlanningStateRequest(_addr);
			psr.m_pprAddr = _addr;
			StateRequestReceiver receiver = new StateRequestReceiver();

			// create a filter to listen for the result of the state
			// being written back... do this before writing, just to be
			// safe
			addChangeFilter(ChangeFilterFactory.createAddressFilter(id, sa,
					WorkingMemoryOperation.OVERWRITE), receiver);

			m_requestFilters.add(receiver);

			addToWorkingMemory(id, sa, psr);

			log("CPC: written state request to: " + sa);
		}

	}

	/**
	 * @param _preparing
	 * @throws SubarchitectureProcessException
	 */
	private void setRequestStatus(PlanningStatus _preparing, TriBool _successful)
			throws SubarchitectureProcessException {

		assert m_pprAddr != null : "this should be set if processing is happening";

		// get from wm
		PlanningProcessRequest ppr = (PlanningProcessRequest) getWorkingMemoryEntry(
				m_pprAddr).getData();

		// change status
		ppr.m_status = _preparing;
		ppr.m_succeeded = _successful;

		// overwrite
		overwriteWorkingMemory(m_pprAddr.m_id, m_pprAddr.m_subarchitecture, ppr);
	}
	/**
	 * @param _preparing
	 * @throws SubarchitectureProcessException
	 */
	private void setRequestStatus(PlanningStatus _preparing)
			throws SubarchitectureProcessException {

		setRequestStatus(_preparing, TriBool.triIndeterminate);
	}

	/**
	 * Write a particular plan step to working memory.. This can be overridden
	 * by subclasses to specialise the actions written.
	 * 
	 * @param _step
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	protected WorkingMemoryPointer writeActionStep(Command _step)
			throws SubarchitectureProcessException {
		// this is going to be a bit odd for the time being, just to
		// test

		String actionSA = getActionSubarchitecture(_step.mapl_action.name);

		log("CPC: writing new action step: " + actionSA);

		// might need to be subclassed to know exactly what actions
		// should be taken, or maybe this is enough

		// just write the step to working memory
		String stepID = newDataID();

		addToWorkingMemory(stepID, actionSA, _step, OperationMode.BLOCKING);

		log("written step: " + PlanningUtils.toString(_step));

		return new WorkingMemoryPointer(CASTUtils.typeName(Command.class),
				new WorkingMemoryAddress(stepID, actionSA));
	}

	/**
	 * @param _step
	 * @throws SubarchitectureProcessException
	 */
	protected void triggerExecution(Command _step)
			throws SubarchitectureProcessException {
		WorkingMemoryPointer ptr = writeActionStep(_step);
		writeActionStruct(ptr);
	}

	/**
	 * @param _ptr
	 * @throws SubarchitectureProcessException
	 */
	protected void writeActionStruct(WorkingMemoryPointer _ptr)
			throws SubarchitectureProcessException {

		assert(m_pprCause != null);
		
		Action action = new Action(_ptr, m_pprCause, PlanningStatus.PROPOSED,
				TriBool.triIndeterminate, new PlanningStateLists(
						new ObjectDeclaration[0], new Fact[0]),
				new PlanningStateLists(new ObjectDeclaration[0], new Fact[0]));

		// add a receiver to listen for completion of the action
		String actionID = newDataID();

		addChangeFilter(ChangeFilterFactory.createAddressFilter(actionID,
				_ptr.m_address.m_subarchitecture,
				WorkingMemoryOperation.OVERWRITE), new ActionReceiver());

		log("CPC: listening for overwrite at: " + actionID + " in sa "
				+ _ptr.m_address.m_subarchitecture);

		// add action
		addToWorkingMemory(actionID, _ptr.m_address.m_subarchitecture, action);
	}

	/**
	 * @param _action
	 * @return
	 * @throws PlanningSubarchitectureException
	 */
	protected String getActionSubarchitecture(String _action)
			throws PlanningSubarchitectureException {
		String actSA = m_actionSubachMap.get(_action);
		if (actSA == null) {
			throw new PlanningSubarchitectureException(
					"no subarch for action: " + _action);
		}

		return actSA;
	}

	/**
	 * 
	 */
	private void waitInRunComponent() {

		try {
			// log("CPC: sleeping");
			m_runComponentLock.wait();
			// log("CPC: awake");
		}
		catch (InterruptedException e) {
			e.printStackTrace();
			System.exit(1);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#runComponent()
	 */
	@Override
	protected void runComponent() {
		while (m_status == ProcessStatus.RUN) {

			synchronized (m_runComponentLock) {
				waitInRunComponent();
			}

			// log("CPC: runcomponent: woken up, ready for execution");

			try {
				// now go into execution loop
				int replanCount = 0;

				while (m_currentTaskID != null) {

					// check whether the plan achieve
					if (m_cpp.goalAchieved()) {
						log("CPC: goal achieved");
						executionSucceeded();
					}
					else if (!m_cpp.goalAcheivable()) {
						log("CPC: plan is not executable");
						executionFailed();
					}
					else {

						if (m_cpp.replanningTriggered()) {
							log("CPC: replanning was triggered");
							// if we've replanned too much, give up
							if (++replanCount > m_replanLimit) {
								log("CPC: replanning count exceeded -- failing");
								executionFailed();
								break;
							}
						}

						// continue
						Command step = m_cpp.getNextStep();

						// allow some sanity checking
						m_lastCompletedAction = null;
						log("CPC: executing plan step: "
								+ PlanningUtils.toString(step));

						// execute action
						synchronized (m_runComponentLock) {
							triggerExecution(step);
							waitInRunComponent();
						}

						assert (m_lastCompletedAction != null);

						// regenerate state after action
						synchronized (m_runComponentLock) {
							// ask subarchs for state entries
							requestCurrentState();
							// block here until all responses received
							waitInRunComponent();
							// log("CPC: runcomponent: state built");
						}

						// now check what happened
						boolean success = (m_lastCompletedAction.m_succeeded == TriBool.triTrue);

						// reset replanning count if we did something
						// successfully
						if (success) {
							replanCount = 0;
						}

						PlanningStateLists observedState = m_state
								.toPlanningState();

						m_cpp.stepTaken(step, success, observedState.m_objects,
								observedState.m_facts,
								m_lastCompletedAction.m_addList,
								m_lastCompletedAction.m_deleteList);

						log("Planner state after action:");

						log("Objects:");
						for (ObjectDeclaration obj : m_cpp.currentObjects()) {
							log(PlanningUtils.toString(obj));
						}
						log("Facts:");
						for (Fact fct : m_cpp.currentFacts()) {
							log(PlanningUtils.toString(fct));
						}

						
						
						log("Observed state after action:");
						log("Objects:");
						for (ObjectDeclaration obj : observedState.m_objects) {
							log(PlanningUtils.toString(obj));
						}
						log("Facts:");
						for (Fact fct : observedState.m_facts) {
							log(PlanningUtils.toString(fct));
						}
					}
				}
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				System.exit(1);
			}
			catch (PlanningException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				System.exit(1);
			}

		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {
		// log("CPC: taskAdopted(): " + _taskID);
		try {
			if (_taskID.startsWith(BUILD_INITIAL_STATE)) {
				m_currentTaskID = _taskID;
				// log("CPC: taskAdopted(): building state...");
				buildStateTask();
			}
			else if (_taskID.startsWith(CREATE_PLAN)) {
				m_currentTaskID = _taskID;
				// log("CPC: taskAdopted(): creating plan...");
				createPlanTask();
			}
			else if (_taskID.startsWith(EXECUTE_PLAN)) {
				m_currentTaskID = _taskID;
				// log("CPC: taskAdopted(): executing plan...");
				executePlanTask();
			}
			else {
				// log("CPC: taskAdopted(): unknown task ID...");
			}

		}
		catch (Exception e) {
			e.printStackTrace();
			currentTaskFailed();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
		WorkingMemoryChange wmc = m_taskMap.remove(_taskID);
		if (wmc == null) {
			println("taskRejected: received unknown taskid: " + _taskID);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		String server = _config.getProperty("--server-exe");

		if (server == null) {
			server = _config.getProperty("-s");
		}

		if (server != null) {
			log("CPC:setting planner server exectutable to: " + server);
			PlannerServerFactory.setPlannerExecutablePath(server);
		}

		String replanLimit = _config.getProperty("--replan");
		if (replanLimit == null) {
			replanLimit = _config.getProperty("-r");
		}

		if (replanLimit != null) {
			m_replanLimit = Integer.parseInt(replanLimit);
		}

		log("CPC:replan limit: " + m_replanLimit);
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
			// listen for process requests
			log("CPC: start: registering change filters...");

			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					PlanningProcessRequest.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							String taskID = proposeTask(BUILD_INITIAL_STATE);
							m_taskMap.put(taskID, _wmc);
						}
					});

			// add registerations
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					ActionRegistration.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {/*
														 * (non-Javadoc)
														 * 
														 * @see cast.architecture.abstr.WorkingMemoryChangeReceiver#workingMemoryChanged(cast.cdl.WorkingMemoryChange)
														 */
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							registerAction(_wmc.m_address);
						}
					});
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param _address
	 */
	private void registerAction(WorkingMemoryAddress _address) {
		// log("CPC: register action WMA...");
		try {
			ActionRegistration reg = (ActionRegistration) getWorkingMemoryEntry(
					_address).getData();
			registerAction(reg);

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			System.exit(1);
		}

	}

	/**
	 * @param _reg
	 */
	protected void registerAction(ActionRegistration _reg) {
		m_actionSubachMap.put(_reg.m_action, _reg.m_subarchitecture);
		log("CPC: registered action: " + _reg.m_action + " for subarch "
				+ _reg.m_subarchitecture);
	}

}
