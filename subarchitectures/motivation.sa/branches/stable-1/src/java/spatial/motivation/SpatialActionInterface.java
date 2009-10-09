/**
 * 
 */
package spatial.motivation;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;

import SpatialData.AVSAction;
import SpatialData.AVSCommand;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.StatusError;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.ActiveVisualSearch;
import execution.slice.actions.ExplorePlace;
import execution.slice.actions.GoToPlace;
import execution.util.ActionExecutor;
import execution.util.ActionExecutorFactory;
import execution.util.LocalActionStateManager;

/**
 * Component to listen to planner actions the trigger the spatial sa as
 * appropriate.
 * 
 * @author nah
 * 
 */
public class SpatialActionInterface extends ManagedComponent {

	private LocalActionStateManager m_actionStateManager;

	private HashSet<Long> m_placeIDs;

	/**
	 * AVS timeout. If less than or equal to 0 there is no timeout. Default to 5
	 * minutes.
	 */
	private long m_avsTimeoutMillis = 300000;

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

	private class AVSExecutor extends Thread implements ActionExecutor {

		private long[] m_avsPlaceIDs;
		private ExecutionCompletionCallback m_callback;
		private WorkingMemoryAddress m_avsAddr;
		private AVSCommand m_avsCmd;
		private boolean m_isStopped;

		public AVSExecutor() {
			m_isStopped = false;
		}

		@Override
		public boolean accept(Action _action) {
			m_avsPlaceIDs = ((ActiveVisualSearch) _action).placeIDs;
			return true;
		}

		@Override
		public TriBool execute() {
			return null;
		}

		@Override
		public void execute(ExecutionCompletionCallback _callback) {

			log("running AVS on places: " + Arrays.toString(m_avsPlaceIDs));

			// avs never returns, so just just keep listening
			m_avsCmd = new AVSCommand(m_avsPlaceIDs, AVSAction.PLAN);
			m_avsAddr = new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID());
			try {
				addToWorkingMemory(m_avsAddr, m_avsCmd);
				m_callback = _callback;
				if (useAVSTimeout()) {
					start();
				}
			} catch (CASTException e) {
				println(e.message);
				e.printStackTrace();
				_callback.executionComplete(TriBool.TRIFALSE);
			}
		}

		@Override
		public boolean isBlockingAction() {
			return false;
		}

		@Override
		public void run() {
			// sleep for the timeout
			if (useAVSTimeout()) {
				try {
					Thread.sleep(m_avsTimeoutMillis);
					if (!m_isStopped) {
						log("halting AVS after timeout");
						// stop avs
						m_avsCmd.cmd = AVSAction.STOPAVS;
						overwriteWorkingMemory(m_avsAddr, m_avsCmd);
						m_callback.executionComplete(TriBool.TRITRUE);
					}
				} catch (InterruptedException e) {
					e.printStackTrace();
				} catch (DoesNotExistOnWMException e) {
					e.printStackTrace();
				} catch (ConsistencyException e) {
					// TODO Auto-generated catch block
				} catch (PermissionException e) {
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					e.printStackTrace();
				}
			}
		}

		@Override
		public void stopExecution() {
			try {
				m_isStopped = true;
				m_avsCmd.cmd = AVSAction.STOPAVS;
				overwriteWorkingMemory(m_avsAddr, m_avsCmd);
			} catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			} catch (ConsistencyException e) {
				e.printStackTrace();
			} catch (PermissionException e) {
				e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				e.printStackTrace();
			}

		}

	}

	private class GoToPlaceExecutor implements ActionExecutor,
			WorkingMemoryChangeReceiver {

		private long m_placeID;
		private ExecutionCompletionCallback m_callback;
		private WorkingMemoryAddress m_navCmdAddr;
		private boolean m_isComplete = false;

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
				println(e.message);
				e.printStackTrace();
				_callback.executionComplete(TriBool.TRIFALSE);
			}
		}

		/**
		 * Sets all values necessary to prevent exceptions later on
		 */
		private NavCommand newNavCommand() {
			return new NavCommand(CommandType.STOP, Priority.NORMAL, null,
					null, null, null, null, StatusError.NONE,
					Completion.COMMANDPENDING);
		}

		public boolean isBlockingAction() {
			return false;
		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {

			// read in the nav cmd
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
			}
		}

		@Override
		public void stopExecution() {
			// remove overwrite receiver
			if (!m_isComplete) {
				try {
					log("aborting execution");
					removeChangeFilter(this);
					// reread
					NavCommand navCmd = getMemoryEntry(m_navCmdAddr,
							NavCommand.class);
					navCmd.comp = Completion.COMMANDABORTED;
					overwriteWorkingMemory(m_navCmdAddr, navCmd);

				} catch (DoesNotExistOnWMException e) {
					//
					// e.printStackTrace();
				} catch (SubarchitectureComponentException e) {
					e.printStackTrace();
				}
			}
		}
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

		m_actionStateManager.registerActionType(ActiveVisualSearch.class,
				new ActionExecutorFactory() {
					@Override
					public ActionExecutor getActionExecutor() {
						return new AVSExecutor();
					}
				});

		m_actionStateManager.registerActionType(ExplorePlace.class,
				new ActionExecutorFactory() {
					@Override
					public ActionExecutor getActionExecutor() {
						return new AlwaysSucceedsExecutor();
					}
				});

		// add a listener to check for place ids, for checking purposes
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				Place place = getMemoryEntry(_wmc.address, Place.class);

				// store id
				addPlace(place);
			}

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
		});

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
	}

	/**
	 * @return
	 */
	private boolean useAVSTimeout() {
		return m_avsTimeoutMillis > 0;
	}

	public SpatialActionInterface() {
	}

}
