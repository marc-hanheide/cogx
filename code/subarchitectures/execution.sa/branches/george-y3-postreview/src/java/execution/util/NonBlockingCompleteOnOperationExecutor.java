package execution.util;

import Ice.ObjectImpl;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.Action;
import execution.slice.TriBool;

/**
 * Add a command to wm and then report that action exection is complete when a
 * particular wm operation happens to that command. This assumes that the stated
 * operation always represents success.
 * 
 * @author nah
 * 
 * @param <ActionType>
 */
public abstract class NonBlockingCompleteOnOperationExecutor<ActionType extends Action>
		extends NonBlockingActionExecutor<ActionType> implements
		WorkingMemoryChangeReceiver {

	protected String m_cmdID;

	public NonBlockingCompleteOnOperationExecutor(ManagedComponent _component,
			Class<ActionType> _actCls) {
		super(_component, _actCls);
	}

	protected <T extends ObjectImpl> void addThenCompleteOnDelete(T _command) {
		addThenCompleteOnDelete(getComponent().newDataID(), _command);
	}

	protected <T extends ObjectImpl> void addThenCompleteOnDelete(String _id,
			T _command) {
		addThenCompleteOnOperation(_id, _command, WorkingMemoryOperation.DELETE);
	}

	protected <T extends ObjectImpl> void addThenCompleteOnOverwrite(T _command) {
		addThenCompleteOnOverwrite(getComponent().newDataID(), _command);
	}

	protected <T extends ObjectImpl> void addThenCompleteOnOverwrite(
			String _id, T _command) {
		addThenCompleteOnOperation(_id, _command,
				WorkingMemoryOperation.OVERWRITE);
	}

	protected <T extends ObjectImpl> void addThenCompleteOnOperation(
			String _id, T _command, WorkingMemoryOperation _operation) {
		assert (_operation == WorkingMemoryOperation.DELETE || _operation == WorkingMemoryOperation.OVERWRITE);

		m_cmdID = _id;
		getComponent().addChangeFilter(
				ChangeFilterFactory.createIDFilter(m_cmdID, _operation), this);
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

	/**
	 * Called when the action has completed to allow any cleanup. If you need to
	 * check a completion status you should be using
	 * NonBlockingCompleteFromStatusExecutor instead.
	 */
	protected void actionComplete() {
	}

	/**
	 * Retreive the input command object from WM.
	 * 
	 * @param <T>
	 * @param _cmdCls
	 * @return
	 * @throws DoesNotExistOnWMException
	 */
	protected <T extends ObjectImpl> T getCommandObject(Class<T> _cmdCls)
			throws DoesNotExistOnWMException {
		if (m_cmdID == null) {
			throw new RuntimeException("Command not set yet");
		}
		return getComponent().getMemoryEntry(m_cmdID, _cmdCls);
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
		actionComplete();
		executionComplete(TriBool.TRITRUE);
	}

}
