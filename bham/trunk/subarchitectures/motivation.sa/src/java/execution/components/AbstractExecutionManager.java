/**
 * 
 */
package execution.components;

import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
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

/**
 * Superclass for execution managing component.
 * 
 * @author nah
 * 
 */
public abstract class AbstractExecutionManager extends ManagedComponent {

	private class ActionMonitor implements WorkingMemoryChangeReceiver {
		private final WorkingMemoryAddress m_actionAddress;

		public ActionMonitor(WorkingMemoryAddress _actionAddress) {
			m_actionAddress = _actionAddress;
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

	protected void triggerExecution(Action _action)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		// new address for the actuibs
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
				getSubarchitectureID());
		// create a monitor object for the action
		new ActionMonitor(wma);
		// add the action to wm
		addToWorkingMemory(wma, _action);
	}

}
