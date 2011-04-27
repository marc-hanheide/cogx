/**
 * 
 */
package execution.components;

import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import execution.slice.Action;
import execution.slice.ActionStatus;
import execution.slice.TriBool;
import execution.util.ActionMonitor;

/**
 * Superclass for execution managing component.
 * 
 * @author nah
 * 
 */
public abstract class AbstractExecutionManager extends ManagedComponent {

	public abstract boolean isPaused();
	
	private class ActionCallback implements WorkingMemoryChangeReceiver {
		private final WorkingMemoryAddress m_actionAddress;
		private final ActionMonitor m_monitor;

		public ActionCallback(WorkingMemoryAddress _actionAddress,
				ActionMonitor _monitor) {
			m_actionAddress = _actionAddress;
			m_monitor = _monitor;
			addChangeFilter(ChangeFilterFactory.createAddressFilter(
					m_actionAddress, WorkingMemoryOperation.OVERWRITE), this);
		}

		/**
		 * Should only receive overwrotes for the action it's monitoring.
		 */
		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {

			assert (_wmc.address.equals(m_actionAddress));

			Action action = getMemoryEntry(m_actionAddress, Action.class);

	
			if (action.status == ActionStatus.COMPLETE) {
				log(CASTUtils.concatenate("Action COMPLETE at ", CASTUtils
						.toString(m_actionAddress), " succeeded: ",
						action.success));

				m_monitor.actionComplete(action);

				// remove action from wm
				deleteFromWorkingMemory(m_actionAddress);
				// and remove self from listeners
				removeChangeFilter(this);
			} else {
				log(CASTUtils.concatenate("Action at ", CASTUtils
						.toString(m_actionAddress), " is ", action.status));

			}

		}
	}

	public WorkingMemoryAddress triggerExecution(Action _action,
			ActionMonitor _monitor) throws AlreadyExistsOnWMException,
			DoesNotExistOnWMException, UnknownSubarchitectureException {
		// new address for the action
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
				getSubarchitectureID());
		// create a monitor object for the action
		new ActionCallback(wma, _monitor);
		// add the action to wm
		addToWorkingMemory(wma, _action);
		return wma;
	}

	public void stopExecution(WorkingMemoryAddress _actionAddress)
			throws DoesNotExistOnWMException, PermissionException,
			UnknownSubarchitectureException {
		deleteFromWorkingMemory(_actionAddress);
	}

	/**
	 * 
	 * Create a new action of a given type. Set members to non crashy defaults.
	 * 
	 * @param <ActionType>
	 * @param _actionClass
	 * @return
	 * @throws CASTException
	 */
	protected final <ActionType extends Action> ActionType newActionInstance(
			Class<ActionType> _actionClass) throws CASTException {
		try {
			ActionType action = _actionClass.newInstance();
			action.status = ActionStatus.PENDING;
			action.success = TriBool.TRIINDETERMINATE;
			return action;
		} catch (Exception e) {
			CASTException ce = new CASTException(
					"Unable to create action for class: " + _actionClass);
			ce.initCause(e);
			throw ce;
		}
	}

}
