package castutils.meta;

import java.util.Collection;

import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.WorkingMemoryWriterComponent;
import castutils.slice.WMAdd;
import castutils.slice.WMDelete;
import castutils.slice.WMOperation;
import castutils.slice.WMOverwrite;

public class OperationPerformer {

	private final WorkingMemoryWriterComponent m_writer;

	public OperationPerformer(WorkingMemoryWriterComponent _writer) {
		m_writer = _writer;
	}

	public void performOperation(WMAdd _add) throws AlreadyExistsOnWMException,
			DoesNotExistOnWMException, UnknownSubarchitectureException {
		m_writer.addToWorkingMemory(_add.address, _add.entry);
	}

	public void performOperation(WMOverwrite _ovr)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		m_writer.overwriteWorkingMemory(_ovr.address, _ovr.entry);
	}

	public void performOperation(WMDelete _del)
			throws DoesNotExistOnWMException, PermissionException,
			UnknownSubarchitectureException {
		m_writer.deleteFromWorkingMemory(_del.address);
	}

	public void performOperations(Collection<? extends WMOperation> _operations)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {
		for (WMOperation op : _operations) {
			if (op instanceof WMAdd) {
				performOperation((WMAdd) op);
			} else if (op instanceof WMOverwrite) {
				performOperation((WMOverwrite) op);
			} else if (op instanceof WMDelete) {
				performOperation((WMDelete) op);
			}
		}
	}

}
