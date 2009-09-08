/**
 * 
 */
package spatial.motivation;

import java.util.HashSet;

import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.StatusError;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.Action;
import execution.slice.TriBool;
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

	private class GoToPlaceExecutor implements ActionExecutor,
			WorkingMemoryChangeReceiver {

		private long m_placeID;
		private ExecutionCompletionCallback m_callback;

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
			WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID());
			addChangeFilter(ChangeFilterFactory.createAddressFilter(wma,
					WorkingMemoryOperation.OVERWRITE), this);
			m_callback = _callback;

			try {
				addToWorkingMemory(wma, cmd);

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
			if (cmd.comp == Completion.COMMANDABORTED
					|| cmd.comp == Completion.COMMANDFAILED) {
				m_callback.executionComplete(TriBool.TRIFALSE);
				deleteFromWorkingMemory(_wmc.address);
				removeChangeFilter(this);
			} else if (cmd.comp == Completion.COMMANDSUCCEEDED) {
				m_callback.executionComplete(TriBool.TRITRUE);
				deleteFromWorkingMemory(_wmc.address);
				removeChangeFilter(this);
			} else {
				log("command in progress: " + cmd.comp);
			}
		}
	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		m_actionStateManager.registerActionType(GoToPlace.class,
				new ActionExecutorFactory() {
					public ActionExecutor getActionExecutor() {
						return new GoToPlaceExecutor();
					}
				});

		// add a listener to check for place ids, for checking purposes
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				Place place = getMemoryEntry(_wmc.address, Place.class);

				// lazy creation
				if (m_placeIDs == null) {
					m_placeIDs = new HashSet<Long>();
				}

				// store id
				m_placeIDs.add(place.id);
				log("stored id: " + place.id);
			}
		});

	}

}
