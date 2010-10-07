/**
 * 
 */
package spatial.execution;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import SpatialData.AVSStatus;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.ProcessViewPointCommand;
import SpatialData.StatusError;
import SpatialData.ViewPoint;
import SpatialData.ViewPointGenerationCommand;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.CreateConesForModel;
import execution.slice.actions.ExplorePlace;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.ProcessCone;
import execution.util.ActionExecutor;
import execution.util.ActionExecutorFactory;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteOnOperationExecutor;

/**
 * Component to listen to planner actions the trigger the spatial sa as
 * appropriate.
 * 
 * @author nah
 * 
 */
public class SpatialActionInterface extends ManagedComponent {
	private class AlwaysSucceedsExecutor implements ActionExecutor {

		@Override
		public boolean accept(Action _action) {
			return true;
		}

		@Override
		public TriBool execute() {
			return TriBool.TRITRUE;
		}

		@Override
		public void execute(ExecutionCompletionCallback _callback) {

		}

		@Override
		public boolean isBlockingAction() {
			return true;
		}

		@Override
		public void stopExecution() {
		}

	}

	public static class ViewConeGenerationExecutor extends
			NonBlockingCompleteOnOperationExecutor<CreateConesForModel> {

		public ViewConeGenerationExecutor(ManagedComponent _component) {
			super(_component, CreateConesForModel.class);
		}

		@Override
		protected boolean acceptAction(CreateConesForModel _action) {
			return true;
		}

		@Override
		public void executeAction() {
			// first delete all cones from previous object call
			((SpatialActionInterface) getComponent())
					.removeObjectViewPoints(getAction().model);

			// the generate new ones
			ViewPointGenerationCommand cmd = new ViewPointGenerationCommand(
					getAction().model, getAction().placeIDs,
					AVSStatus.INPROGRESS);
			addThenCompleteOnOverwrite(newWorkingMemoryAddress(), cmd);
		}
	}

	public static class ViewConeProcessExecutor extends
			NonBlockingCompleteOnOperationExecutor<ProcessCone> {

		private ViewPoint m_vp;

		public ViewConeProcessExecutor(ManagedComponent _component) {
			super(_component, ProcessCone.class);
		}

		@Override
		protected boolean acceptAction(ProcessCone _action) {
			try {
				m_vp = getComponent().getMemoryEntry(_action.coneAddress,
						ViewPoint.class);
			} catch (CASTException e) {
				logException(e);
				return false;
			}
			return true;
		}

		@Override
		public void executeAction() {
			ProcessViewPointCommand cmd = new ProcessViewPointCommand(
					AVSStatus.INPROGRESS, m_vp, new String[] { m_vp.label });
			addThenCompleteOnOverwrite(newWorkingMemoryAddress(), cmd);
		}
	}

	
	//not used in year 2
//	private class AVSExecutor extends Thread implements ActionExecutor,
//			WorkingMemoryChangeReceiver {
//
//		private WorkingMemoryAddress m_avsAddr;
//		private AVSCommand m_avsCmd;
//		private long[] m_avsPlaceIDs;
//		private ExecutionCompletionCallback m_callback;
//		private boolean m_isStopped;
//
//		public AVSExecutor() {
//			m_isStopped = false;
//		}
//
//		@Override
//		public boolean accept(Action _action) {
//			m_avsPlaceIDs = ((ActiveVisualSearch) _action).placeIDs;
//			return true;
//		}
//
//		@Override
//		public TriBool execute() {
//			return null;
//		}
//
//		@Override
//		public void execute(ExecutionCompletionCallback _callback) {
//
//			log("running AVS on places: " + Arrays.toString(m_avsPlaceIDs));
//
//			// avs never returns, so just just keep listening
//			m_avsCmd = new AVSCommand(m_avsPlaceIDs, AVSAction.PLAN);
//			m_avsAddr = new WorkingMemoryAddress(newDataID(),
//					getSubarchitectureID());
//			try {
//				addChangeFilter(ChangeFilterFactory.createAddressFilter(
//						m_avsAddr, WorkingMemoryOperation.DELETE), this);
//				addToWorkingMemory(m_avsAddr, m_avsCmd);
//				m_callback = _callback;
//				if (useAVSTimeout()) {
//					start();
//				}
//			} catch (CASTException e) {
//				println(e.message);
//				e.printStackTrace();
//				_callback.executionComplete(TriBool.TRIFALSE);
//			}
//		}
//
//		@Override
//		public boolean isBlockingAction() {
//			return false;
//		}
//
//		@Override
//		public void run() {
//			// sleep for the timeout
//			if (useAVSTimeout()) {
//				try {
//					Thread.sleep(m_avsTimeoutMillis);
//					if (!m_isStopped) {
//						log("halting AVS after timeout");
//						// stop avs
//						m_avsCmd.cmd = AVSAction.STOPAVS;
//						overwriteWorkingMemory(m_avsAddr, m_avsCmd);
//						removeChangeFilter(this);
//						m_callback.executionComplete(TriBool.TRITRUE);
//					}
//				} catch (InterruptedException e) {
//					e.printStackTrace();
//				} catch (DoesNotExistOnWMException e) {
//					e.printStackTrace();
//				} catch (ConsistencyException e) {
//					e.printStackTrace();
//				} catch (PermissionException e) {
//					e.printStackTrace();
//				} catch (UnknownSubarchitectureException e) {
//					e.printStackTrace();
//				} catch (SubarchitectureComponentException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			}
//		}
//
//		@Override
//		public void stopExecution() {
//			if (!m_isStopped) {
//				try {
//					m_isStopped = true;
//					m_avsCmd.cmd = AVSAction.STOPAVS;
//					overwriteWorkingMemory(m_avsAddr, m_avsCmd);
//					removeChangeFilter(this);
//
//				} catch (DoesNotExistOnWMException e) {
//					e.printStackTrace();
//				} catch (ConsistencyException e) {
//					e.printStackTrace();
//				} catch (PermissionException e) {
//					e.printStackTrace();
//				} catch (UnknownSubarchitectureException e) {
//					e.printStackTrace();
//				} catch (SubarchitectureComponentException e) {
//					e.printStackTrace();
//				}
//			}
//		}
//
//		@Override
//		public void workingMemoryChanged(WorkingMemoryChange _arg0)
//				throws CASTException {
//			// only called when command has been deleted, i.e. avs has finished
//			// for some reason
//			if (!m_isStopped) {
//				println("AVS finished on its own");
//				m_isStopped = true;
//				m_callback.executionComplete(TriBool.TRITRUE);
//				try {
//					removeChangeFilter(this);
//				} catch (SubarchitectureComponentException e) {
//					e.printStackTrace();
//				}
//			}
//		}
//
//	}

	private class GoToPlaceExecutor implements ActionExecutor,
			WorkingMemoryChangeReceiver {

		private ExecutionCompletionCallback m_callback;
		private boolean m_isComplete = false;
		private WorkingMemoryAddress m_navCmdAddr;
		private long m_placeID;

		public boolean accept(Action _action) {
			m_placeID = ((GoToPlace) _action).placeID;
			return true;
		}

		public TriBool execute() {
			return null;
		}

		public void execute(ExecutionCompletionCallback _callback) {
			// if we haven't seen this place, then fail
			if (!m_placeIDs.contains(m_placeID)) {
				_callback.executionComplete(TriBool.TRIFALSE);
				return;
			}

			// else create the command and send it off, ignoring path transition
			// probs for now
			NavCommand cmd = newNavCommand();
			cmd.cmd = CommandType.GOTOPLACE;
			cmd.destId = new long[1];
			cmd.destId[0] = m_placeID;

			// going to add locally for the time being, but using wma to allow
			// this to be changed later
			m_navCmdAddr = new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID());
			addChangeFilter(ChangeFilterFactory.createAddressFilter(
					m_navCmdAddr, WorkingMemoryOperation.OVERWRITE), this);
			m_callback = _callback;

			try {
				addToWorkingMemory(m_navCmdAddr, cmd);
			} catch (CASTException e) {
				logException(e);
				_callback.executionComplete(TriBool.TRIFALSE);
			}
		}

		public boolean isBlockingAction() {
			return false;
		}

		@Override
		public void stopExecution() {
			// remove overwrite receiver
			if (!m_isComplete) {
				try {
					log("aborting execution");
					removeChangeFilter(this);
					// reread
					lockEntry(m_navCmdAddr, WorkingMemoryPermissions.LOCKEDODR);
					NavCommand navCmd = getMemoryEntry(m_navCmdAddr,
							NavCommand.class);
					navCmd.comp = Completion.COMMANDABORTED;
					overwriteWorkingMemory(m_navCmdAddr, navCmd);
					unlockEntry(m_navCmdAddr);

				} catch (DoesNotExistOnWMException e) {
					//
					// e.printStackTrace();
				} catch (SubarchitectureComponentException e) {
					e.printStackTrace();
				}
			}
		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {

			// read in the nav cmd
			lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDODR);
			NavCommand cmd = getMemoryEntry(_wmc.address, NavCommand.class);
			if (cmd.comp == Completion.COMMANDFAILED) {
				log("command failed by the looks of this: " + cmd.comp);
				m_isComplete = true;
				m_callback.executionComplete(TriBool.TRIFALSE);
				deleteFromWorkingMemory(_wmc.address);
				removeChangeFilter(this);
			} else if (cmd.comp == Completion.COMMANDSUCCEEDED) {
				log("command completed by the looks of this: " + cmd.comp);
				m_isComplete = true;
				m_callback.executionComplete(TriBool.TRITRUE);
				deleteFromWorkingMemory(_wmc.address);
				removeChangeFilter(this);
			} else {
				log("command in progress: " + cmd.comp);
				unlockEntry(_wmc.address);
			}
		}
	}

	public class LookForPeopleExecutorFactory implements ActionExecutorFactory {

		private final ManagedComponent m_component;

		public LookForPeopleExecutorFactory(ManagedComponent _component) {
			m_component = _component;
		}

		@Override
		public ActionExecutor getActionExecutor() {
			return new LookForPeopleExecutor(m_component, m_detections);
		}

	}

	public class LookForObjectsExecutorFactory implements ActionExecutorFactory {

		private final ManagedComponent m_component;

		public LookForObjectsExecutorFactory(ManagedComponent _component) {
			m_component = _component;
		}

		@Override
		public ActionExecutor getActionExecutor() {
			return new LookForObjectsExecutor(m_component, m_detections);
		}

	}

	/**
	 * Sets all values necessary to prevent exceptions later on
	 */
	public static NavCommand newNavCommand() {
		return new NavCommand(CommandType.STOP, Priority.NORMAL, null, null,
				null, null, null, StatusError.NONE, Completion.COMMANDPENDING);
	}

	private LocalActionStateManager m_actionStateManager;

	/**
	 * AVS timeout. If less than or equal to 0 there is no timeout. Default to
	 * 0.
	 * 
	 */
	private long m_avsTimeoutMillis = 0;

	private HashSet<Long> m_placeIDs;

	private int m_detections;

	private HashMap<String, HashSet<WorkingMemoryAddress>> m_viewPoints;

	public SpatialActionInterface() {
		m_detections = 4;
	}

	@Override
	protected void configure(Map<String, String> _config) {
		String avsTimeoutString = _config.get("--avs-timeout");
		if (avsTimeoutString != null) {
			m_avsTimeoutMillis = Long.parseLong(avsTimeoutString);
		}

		if (useAVSTimeout()) {
			log("using AVS timeout of: " + m_avsTimeoutMillis + "ms");
		} else {
			log("no AVS timeout");
		}

		String numDetections = _config.get("--detections");
		if (numDetections != null) {
			m_detections = Integer.parseInt(numDetections);
		}
		log("when looking for people I will run " + m_detections
				+ " detections");

	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		m_actionStateManager.registerActionType(GoToPlace.class,
				new ActionExecutorFactory() {
					@Override
					public ActionExecutor getActionExecutor() {
						return new GoToPlaceExecutor();
					}
				});

//		m_actionStateManager.registerActionType(ActiveVisualSearch.class,
//				new ActionExecutorFactory() {
//					@Override
//					public ActionExecutor getActionExecutor() {
//						return new AVSExecutor();
//					}
//				});

		m_actionStateManager.registerActionType(ExplorePlace.class,
				new ActionExecutorFactory() {
					@Override
					public ActionExecutor getActionExecutor() {
						return new AlwaysSucceedsExecutor();
					}
				});

		m_actionStateManager.registerActionType(LookForObjects.class,
				new LookForObjectsExecutorFactory(this));
		m_actionStateManager.registerActionType(LookForPeople.class,
				new LookForPeopleExecutorFactory(this));

		m_actionStateManager.registerActionType(CreateConesForModel.class,
				new ComponentActionFactory<ViewConeGenerationExecutor>(this,
						ViewConeGenerationExecutor.class));

		m_actionStateManager.registerActionType(ProcessCone.class,
				new ComponentActionFactory<ViewConeProcessExecutor>(this,
						ViewConeProcessExecutor.class));

		// add a listener to check for place ids, for checking purposes
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			/**
			 * @param _place
			 */
			private void addPlace(Place _place) {
				// lazy creation
				if (m_placeIDs == null) {
					m_placeIDs = new HashSet<Long>();
				}

				m_placeIDs.add(_place.id);
				// log("stored id: " + _place.id);
			}

			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				Place place = getMemoryEntry(_wmc.address, Place.class);

				// store id
				addPlace(place);
			}
		});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ViewPoint.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						addViewPoint(_wmc.address);
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ViewPoint.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						removeViewPoint(_wmc.address);
					}
				});
	}

	private void removeViewPoint(WorkingMemoryAddress _address) {
		if (m_viewPoints != null) {

			Set<Entry<String, HashSet<WorkingMemoryAddress>>> entrySet = m_viewPoints
					.entrySet();
			for (Entry<String, HashSet<WorkingMemoryAddress>> entry : entrySet) {
				if (entry.getValue().remove(_address)) {
					return;
				}
			}
		}

	}

	private void addViewPoint(WorkingMemoryAddress _address) {
		if (m_viewPoints == null) {
			m_viewPoints = new HashMap<String, HashSet<WorkingMemoryAddress>>();
		}
		try {
			String objectLabel = getMemoryEntry(_address, ViewPoint.class).label;
			HashSet<WorkingMemoryAddress> objectSet = m_viewPoints
					.get(objectLabel);
			if (objectSet == null) {
				objectSet = new HashSet<WorkingMemoryAddress>();
				m_viewPoints.put(objectLabel, objectSet);
			}
			objectSet.add(_address);
		} catch (CASTException e) {
			logException(e);
		}
	}

	private void removeObjectViewPoints(String _label) {
		assert (_label != null);
		// needed because the non-block action is run in a thread
		lockComponent();

		if (m_viewPoints != null) {
			HashSet<WorkingMemoryAddress> viewPointAddressess = m_viewPoints
					.remove(_label);
			if (viewPointAddressess == null) {
				println("no stored cones for object label: " + _label);
			} else {
				for (WorkingMemoryAddress addr : viewPointAddressess) {
					try {
						deleteFromWorkingMemory(addr);
					} catch (CASTException e) {
						logException(e);
					}
				}

			}
		} else {
			println("no stored cones for object label: " + _label);
		}

		unlockComponent();

	}

	/**
	 * @return
	 */
	private boolean useAVSTimeout() {
		return m_avsTimeoutMillis > 0;
	}

}
