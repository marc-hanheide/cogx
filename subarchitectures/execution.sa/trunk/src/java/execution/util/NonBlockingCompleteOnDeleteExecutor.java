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
import execution.slice.Action;
import execution.slice.TriBool;

public abstract class NonBlockingCompleteOnDeleteExecutor<ActionType extends Action> extends
		NonBlockingActionExecutor<ActionType> implements WorkingMemoryChangeReceiver {

	protected String m_cmdID;

	public NonBlockingCompleteOnDeleteExecutor(ManagedComponent _component, Class<ActionType> _actCls) {
		super(_component, _actCls);
	}

	protected <T extends ObjectImpl> void addThenCompleteOnDelete(
			WorkingMemoryAddress _workingMemoryAddress, T _command) {
		m_cmdID = getComponent().newDataID();
		getComponent().addChangeFilter(ChangeFilterFactory.createIDFilter(m_cmdID,
				WorkingMemoryOperation.DELETE), this);
		try {
			getComponent().addToWorkingMemory(m_cmdID, _command);
		} catch (AlreadyExistsOnWMException e) {
			getComponent().logException(e);
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
