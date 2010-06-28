package execution.util;

import autogen.Planner.Action;
import autogen.Planner.Completion;
import cast.cdl.WorkingMemoryAddress;
import execution.slice.TriBool;

public class PlannedActionWrapper implements ActionMonitor {

	private final Action m_plannedAction;
	private final execution.slice.Action m_systemAction;
	private boolean m_signalledComplete;
	//the address of the system action on wm
	private  WorkingMemoryAddress m_actionAddress;
	
	public PlannedActionWrapper(Action _plannedAction,
			execution.slice.Action _systemAction) {
		m_plannedAction = _plannedAction;
		m_systemAction = _systemAction;
		m_signalledComplete =false;
	}

	public void setActionAddress(WorkingMemoryAddress _actionAddress) {
		m_actionAddress = _actionAddress;
	}
	
	public WorkingMemoryAddress getActionAddress() {
		assert(m_actionAddress != null);
		return m_actionAddress;
	}
	
	public boolean isInProgress() {
		return m_plannedAction.status == Completion.INPROGRESS;
	}

	public boolean isComplete() {
		return m_plannedAction.status == Completion.SUCCEEDED
				|| m_plannedAction.status == Completion.FAILED;
	}

	/**
	 * Callback which is triggered when the executed action is complete
	 */
	@Override
	public void actionComplete(execution.slice.Action _action) {
		if (_action.success == TriBool.TRITRUE) {
			m_plannedAction.status = Completion.SUCCEEDED;
		} else {
			m_plannedAction.status = Completion.FAILED;
		}
	}

	public TriBool wasSuccessful() {

		switch (m_plannedAction.status) {
		case SUCCEEDED:
			return TriBool.TRITRUE;
		case FAILED:
			return TriBool.TRIFALSE;
		default:
			return TriBool.TRIINDETERMINATE;
		}
	}

	/**
	 * Called when execution is to start
	 * 
	 * @return
	 */
	public execution.slice.Action execute() {
		m_plannedAction.status = Completion.INPROGRESS;
		return m_systemAction;
	}

	public void sentCompletionSignal() {
		m_signalledComplete = true;
	}

	public boolean haveSentCompletionSignal() {
		return m_signalledComplete;
	}

}
