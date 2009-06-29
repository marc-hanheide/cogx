package cast.server;

import Ice.Current;
import Ice.Identity;
import Ice.Object;
import Ice.ObjectAdapter;
import Ice.ObjectPrx;
import cast.ComponentCreationException;
import cast.core.CASTUtils;
import cast.interfaces.CASTComponentPrx;
import cast.interfaces.CASTComponentPrxHelper;
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

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public CASTComponentPrx newComponent(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		CASTComponentPrx prx = CASTComponentPrxHelper.checkedCast(base);
		if (prx == null) {
			throw new ComponentCreationException(_type
					+ " cannot be cast to a CASTComponentPrx");
		}
		return prx;
	}

	public ManagedComponentPrx newManagedComponent(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		ManagedComponentPrx prx = ManagedComponentPrxHelper.checkedCast(base);
		if (prx == null) {
			throw new ComponentCreationException(_type
					+ " cannot be cast to a ManagaedComponentPrx");
		}
		return prx;

	}

	public TaskManagerPrx newTaskManager(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		TaskManagerPrx prx = TaskManagerPrxHelper.checkedCast(base);
		if (prx == null) {
			throw new ComponentCreationException(_type
					+ " cannot be cast to a TaskManagerPrx");
		}
		return prx;

	}

	public UnmanagedComponentPrx newUnmanagedComponent(String _id,
			String _type, Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		UnmanagedComponentPrx prx = UnmanagedComponentPrxHelper
				.checkedCast(base);
		if (prx == null) {
			throw new ComponentCreationException(_type
					+ " cannot be cast to a UmanagedComponentPrx");
		}
		return prx;

	}

	public WorkingMemoryPrx newWorkingMemory(String _id, String _type,
			Current __current) throws ComponentCreationException {
		ObjectPrx base = createBase(_id, _type, __current);
		WorkingMemoryPrx prx = WorkingMemoryPrxHelper.checkedCast(base);
		if (prx == null) {
			throw new ComponentCreationException(_type
					+ " cannot be cast to a WorkingMemoryPrx");
		}
		return prx;

	}

	private ObjectPrx createBase(String _id, String _type, Current __current)
			throws ComponentCreationException {
		try {
			ObjectAdapter adapter = __current.adapter;
			Identity id = new Identity(_id, _type);
			Object servant = CASTUtils.createServant(id, adapter);
			ObjectPrx base = adapter.add(servant, id);
			assert (base != null);
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

	// public void remove(String _id, String _type, Current __current) {
	// System.out.println("CASTComponentFactory.destroy(): " + _id);
	// __current.adapter.remove(new Identity(_id,_type));
	// }

}
