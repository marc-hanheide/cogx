package cast.server;

import Ice.Current;
import Ice.Identity;
import Ice.Object;
import Ice.ObjectAdapter;
import Ice.ObjectPrx;
import cast.ComponentCreationException;
import cast.core.CASTUtils;
import cast.interfaces.CASTComponentPrx;
import cast.interfaces.ManagedComponentPrx;
import cast.interfaces.ManagedComponentPrxHelper;
import cast.interfaces.TaskManagerPrx;
import cast.interfaces.TaskManagerPrxHelper;
import cast.interfaces.UnmanagedComponentPrx;
import cast.interfaces.UnmanagedComponentPrxHelper;
import cast.interfaces.WorkingMemoryPrx;
import cast.interfaces.WorkingMemoryPrxHelper;
import cast.interfaces._ComponentFactoryDisp;

/**
 * 
 * Server that creates components
 * 
 * @author nah
 * 
 */
public class CASTComponentFactory extends _ComponentFactoryDisp {

	public CASTComponentPrx newComponent(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		return WorkingMemoryPrxHelper.checkedCast(base);
	}

	public ManagedComponentPrx newManagedComponent(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		return ManagedComponentPrxHelper.checkedCast(base);
	}

	public TaskManagerPrx newTaskManager(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		return TaskManagerPrxHelper.checkedCast(base);
	}

	public UnmanagedComponentPrx newUnmanagedComponent(String _id,
			String _type, Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		return UnmanagedComponentPrxHelper.checkedCast(base);
	}

	public WorkingMemoryPrx newWorkingMemory(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		return WorkingMemoryPrxHelper.checkedCast(base);
	}

	private ObjectPrx createBase(String _id, String _type, Current __current)
			throws ComponentCreationException {
		try {
			ObjectAdapter adapter = __current.adapter;
			Identity id = new Identity(_id, _type);
			Object servant = CASTUtils.createServant(id, adapter);
			ObjectPrx base = adapter.add(servant, id);
			return base;
		} catch (ClassNotFoundException e) {
			ComponentCreationException cce = new ComponentCreationException(
					"Unable to create component: Cannot find class "
							+ e.getMessage());
			cce.initCause(e);
			throw cce;
		} catch (Exception e) {
			ComponentCreationException cce = new ComponentCreationException(
					"Unable to create component: " + e.getClass() + ": "
							+ e.getMessage());
			cce.initCause(e);
			throw cce;
		}
	}

}
