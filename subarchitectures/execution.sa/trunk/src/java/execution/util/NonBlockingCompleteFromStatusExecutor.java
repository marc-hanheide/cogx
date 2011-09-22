package execution.util;

import Ice.ObjectImpl;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.WMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.Action;
import execution.slice.TriBool;

/**
 * Add a command to wm and then report execution status based on something
 * written in the overwritten command.
 * 
 * @author nah
 * 
 * @param <ActionType>
 */
public abstract class NonBlockingCompleteFromStatusExecutor<ActionType extends Action, CommandType extends Ice.Object>
		extends NonBlockingActionExecutor<ActionType> implements
		WorkingMemoryChangeReceiver {

	private final Class<CommandType> m_cmdCls;

	public NonBlockingCompleteFromStatusExecutor(ManagedComponent _component,
			Class<ActionType> _actCls, Class<CommandType> _cmdCls) {
		super(_component, _actCls);
		m_cmdCls = _cmdCls;
	}

	protected <T extends ObjectImpl> void addThenCompleteOnOverwrite(T _command) {
		addThenCompleteOnOverwrite(getComponent().newDataID(), _command);
	}

	protected <T extends ObjectImpl> void addThenCompleteOnOverwrite(
			String _id, T _command) {

		getComponent().addChangeFilter(
				ChangeFilterFactory.createIDFilter(_id,
						WorkingMemoryOperation.OVERWRITE), this);
		try {
			getComponent().addToWorkingMemory(_id, _command);
		} catch (AlreadyExistsOnWMException e) {
			getComponent().logException(e);
			executionComplete(TriBool.TRIFALSE);
		}
	}

	public Class<CommandType> getCommandClass() {
		return m_cmdCls;
	}
	
	@Override
	public void stopExecution() {
		// can't stop this one
	}

	/**
	 * Returns the status of the completed action.
	 * 
	 * @param _cmd
	 *            The command which was overwritten by whichever component did
	 *            the action.
	 * @return Whether the action was successful or not.
	 */
	protected abstract TriBool executionResult(CommandType _cmd);

	/**
	 * Called when the action has completed to allow any cleanup
	 */
	protected void actionComplete() {

	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
		// allow cleanup
		actionComplete();
		try {
			TriBool result = executionResult(getComponent().getMemoryEntry(
					_wmc.address, getCommandClass()));
			log("Result of " + m_cmdCls + " was " + result);
			executionComplete(result);
		} catch (WMException e) {
			logException(e);
			executionComplete(TriBool.TRIFALSE);
		}
	}

}
