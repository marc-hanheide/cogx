package castutils.components;

import java.io.IOException;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * Executes a command on the command line when a particular WM operation occurs.
 */
public abstract class CommandLineOnOperation<CommandType extends Ice.Object>
		extends ManagedComponent {

	private Class<CommandType> m_class;
	private WorkingMemoryOperation m_operation;

	/**
	 * 
	 * @param _cmd
	 *            Object from WM. Will be null on delete operation.
	 * @return
	 */
	protected abstract ProcessBuilder getProcess(CommandType _cmd);

	public CommandLineOnOperation(WorkingMemoryOperation _operation,
			Class<CommandType> _cls) {
		m_operation = _operation;
		m_class = _cls;
	}

	@Override
	protected void start() {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(m_class,
				m_operation), new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {

				try {
					if (_wmc.operation == WorkingMemoryOperation.DELETE) {
						executeProcess(getProcess(null));
					} else {
						executeProcess(getProcess(getMemoryEntry(_wmc.address,
								m_class)));
					}
				} catch (Exception e) {
					CASTException ce = new CASTException(
							"Failure when executing on command line");
					ce.initCause(e);
					throw ce;
				}

			}
		});
	}

	protected int executeProcess(ProcessBuilder _proc) throws IOException,
			InterruptedException {
		Process pr = _proc.start();
		int status = pr.waitFor();
		log("executed: " + _proc.command() + " returned status: " + status);
		return status;
	}

}
