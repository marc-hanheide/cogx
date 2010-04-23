package execution.util;

import Ice.ObjectImpl;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.TriBool;

public abstract class NonBlockingCompleteOnDeleteExecutor extends
		NonBlockingActionExecutor implements WorkingMemoryChangeReceiver {

	private final ManagedComponent m_component;

	protected String m_cmdID;

	public NonBlockingCompleteOnDeleteExecutor(ManagedComponent _component) {
		m_component = _component;
	}

	protected <T extends ObjectImpl> void addThenCompleteOnDelete(
			WorkingMemoryAddress _workingMemoryAddress, T _command) {
		m_cmdID = m_component.newDataID();
		m_component.addChangeFilter(ChangeFilterFactory.createIDFilter(m_cmdID,
				WorkingMemoryOperation.DELETE), this);
		try {
			m_component.addToWorkingMemory(m_cmdID, _command);
		} catch (AlreadyExistsOnWMException e) {
			m_component.logException(e);
			executionComplete(TriBool.TRIFALSE);
		}
	}

	@Override
	public void stopExecution() {
		// can't stop this one
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
		// always succeed, regardless of actual detections.
		executionComplete(TriBool.TRITRUE);
	}

}
