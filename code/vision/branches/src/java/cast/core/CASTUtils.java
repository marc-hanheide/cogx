/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
/**
 * 
 */
package cast.core;

import java.util.HashMap;

import Ice.Communicator;
import Ice.Current;
import Ice.Identity;
import Ice.ObjectAdapter;
import Ice.ObjectPrx;
import cast.ComponentCreationException;
import cast.architecture.ManagedComponent;
import cast.architecture.SubarchitectureTaskManager;
import cast.architecture.SubarchitectureWorkingMemory;
import cast.architecture.UnmanagedComponent;
import cast.cdl.CASTTime;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.interfaces.CASTComponentPrx;
import cast.interfaces.CASTComponentPrxHelper;
import cast.interfaces.ComponentFactoryPrx;
import cast.interfaces.ComponentFactoryPrxHelper;
import cast.interfaces.ManagedComponentPrx;
import cast.interfaces.TaskManagerPrx;
import cast.interfaces.UnmanagedComponentPrx;
import cast.interfaces.WorkingMemoryPrx;
import cast.interfaces._CASTComponentTie;
import cast.interfaces._ComponentManagerTie;
import cast.interfaces._ManagedComponentTie;
import cast.interfaces._TaskManagerTie;
import cast.interfaces._UnmanagedComponentTie;
import cast.interfaces._WorkingMemoryTie;
import cast.server.CASTComponentManager;

/**
 * @author nah
 */
public class CASTUtils {

	private static HashMap<String, ComponentFactoryPrx> m_factories;

	static {
		m_factories = new HashMap<String, ComponentFactoryPrx>();
	}

	private static ComponentFactoryPrx getComponentFactory(Communicator _ic,
			String _host, int _port) {
		String hash = _host + _port;
		ComponentFactoryPrx factory = m_factories.get(hash);
		if (factory == null) {
			Identity _id = new Identity("ComponentFactory", "ComponentFactory");
			ObjectPrx base = _ic.stringToProxy(_ic.identityToString(_id)
					+ ":default -h " + _host + " -p " + _port);
			factory = ComponentFactoryPrxHelper.checkedCast(base);
		}
		return factory;
	}

	public static WorkingMemoryPrx createWorkingMemory(Identity _id,
			Communicator _ic, String _host, int _port)
			throws ComponentCreationException {
		ComponentFactoryPrx factory = getComponentFactory(_ic, _host, _port);
		return factory.newWorkingMemory(_id.name, _id.category);
	}

	public static TaskManagerPrx createTaskManager(Identity _id,
			Communicator _ic, String _host, int _port)
			throws ComponentCreationException {
		ComponentFactoryPrx factory = getComponentFactory(_ic, _host, _port);
		return factory.newTaskManager(_id.name, _id.category);
	}

	public static ManagedComponentPrx createManagedComponent(Identity _id,
			Communicator _ic, String _host, int _port)
			throws ComponentCreationException {
		ComponentFactoryPrx factory = getComponentFactory(_ic, _host, _port);
		return factory.newManagedComponent(_id.name, _id.category);
	}

	public static UnmanagedComponentPrx createUnmanagedComponent(Identity _id,
			Communicator _ic, String _host, int _port)
			throws ComponentCreationException {
		ComponentFactoryPrx factory = getComponentFactory(_ic, _host, _port);
		return factory.newUnmanagedComponent(_id.name, _id.category);
	}

	public static CASTComponentPrx createCASTComponent(Identity _id,
			Communicator _ic, String _host, int _port)
			throws ComponentCreationException {
		ComponentFactoryPrx factory = getComponentFactory(_ic, _host, _port);
		return factory.newComponent(_id.name, _id.category);
	}

	public static CASTComponentPrx getComponent(Identity _id, Communicator _ic,
			String _host, int _port) {
		ObjectPrx base = _ic.stringToProxy(_ic.identityToString(_id)
				+ ":default -h " + _host + " -p " + _port);

		if (base == null) {
			throw new RuntimeException("Cannot create proxy");
		}

		CASTComponentPrx prx = CASTComponentPrxHelper.checkedCast(base);

		prx.ice_ping();

		// sanity check first up
		assert (prx.getID().equals(_id.name));

		return prx;
	}

//	public static CASTComponentPrx getJavaComponent(Identity _id,
//			Communicator _ic, String _host) {
//		return getComponent(_id, _ic, _host, JAVASERVERPORT.value);
//	}
//
//	public static CASTComponentPrx getCppComponent(Identity _id,
//			Communicator _ic, String _host) {
//		return getComponent(_id, _ic, _host, CPPSERVERPORT.value);
//	}
//
//	public static CASTComponentPrx getCASTComponent(
//			ComponentDescription _config, Communicator _ic) {
//		Identity id = new Identity(_config.componentName, _config.className);
//		CASTComponentPrx prx = null;
//		if (_config.language == ComponentLanguage.CPP) {
//			prx = getCppComponent(id, _ic, _config.hostName);
//		} else if (_config.language == ComponentLanguage.JAVA) {
//			prx = getJavaComponent(id, _ic, _config.hostName);
//		} else {
//			assert false;
//		}
//		assert (prx != null);
//
//		prx.configure(_config.configuration);
//		return prx;
//	}

	// /**
	// * @param _namingHost
	// * @param _millis
	// * @param connectionCollection
	// */
	// private static void runCAST(String _namingHost, long _millis,
	// Collection<ProcessConnection> connectionCollection) {
	// ProcessConnection[] connections = new
	// ProcessConnection[connectionCollection
	// .size()];
	// connectionCollection.toArray(connections);
	//
	// try {
	// ClientUtils.runFramework(_namingHost, "1050", connections, _millis);
	// }
	// catch (FrameworkProcessManagerException e) {
	// e.printStackTrace();
	// System.err.println(e.getLocalizedMessage());
	//
	// System.exit(1);
	// }
	// catch (Throwable t) {
	// t.printStackTrace();
	// System.err.println(t.getLocalizedMessage());
	// System.exit(1);
	// }
	// }

	// /**
	// * @param _outcome
	// * @return
	// */
	// public static String toString(TaskOutcome _outcome) {
	// switch (_outcome.value()) {
	// case TaskOutcome._PROCESSING_COMPLETE :
	// return "complete";
	// case TaskOutcome._PROCESSING_COMPLETE_FAILURE :
	// return "failure";
	// case TaskOutcome._PROCESSING_COMPLETE_SUCCESS :
	// return "success";
	// case TaskOutcome._PROCESSING_INCOMPLETE :
	// return "incomplete";
	// }
	// return "unknown value";
	// }

	/**
	 * @param _wma
	 * @return
	 */
	public static String toString(WorkingMemoryAddress _wma) {
		StringBuffer buf = new StringBuffer("[WMA id = ");
		buf.append(_wma.id);
		buf.append(" : sa = ");
		buf.append(_wma.subarchitecture);
		buf.append("]");
		return buf.toString();
	}

	/**
	 * @param _wma
	 * @return
	 */
	public static String toString(WorkingMemoryChange _wmc) {
		StringBuffer buf = new StringBuffer("[WMC ");
		buf.append(_wmc.src);
		buf.append(" ");
		buf.append(_wmc.type);
		buf.append(" ");
		buf.append(toString(_wmc.address));
		buf.append(" ");
		buf.append(toString(_wmc.operation));
		buf.append("]");
		return buf.toString();
	}

	/**
	 * @param _filter
	 * @return
	 */
	public static String toString(WorkingMemoryChangeFilter _filter) {
		StringBuffer buf = new StringBuffer("[WMCF: ");
		buf.append(toString(_filter.operation));
		buf.append(" ");
		buf.append(_filter.src);
		buf.append(" ");
		buf.append(toString(_filter.address));
		buf.append(" ");
		buf.append(_filter.type);
		buf.append(" ");
		buf.append(toString(_filter.restriction));
		buf.append(" (");
		buf.append(_filter.origin);
		buf.append(")]");
		return buf.toString();
	}

	public static String toString(FilterRestriction _restriction) {
		switch (_restriction.value()) {
		case FilterRestriction._ALLSA:
			return "ALL_SA";
		case FilterRestriction._LOCALSA:
			return "LOCAL_SA";
		}
		throw new RuntimeException("Unknown enum value: "
				+ _restriction.value());

	}

	public static String toString(WorkingMemoryOperation _op) {
		switch (_op.value()) {
		case WorkingMemoryOperation._ADD:
			return "ADD";
		case WorkingMemoryOperation._OVERWRITE:
			return "OVERWRITE";
		case WorkingMemoryOperation._DELETE:
			return "DELETE";
		case WorkingMemoryOperation._GET:
			return "GET";
		default:
			return "UNKNOWN";
		}
	}

	// /**
	// * @param _ptr
	// * @return
	// */
	// public static String toString(WorkingMemoryPointer _ptr) {
	// StringBuffer buf = new StringBuffer("[WMP: ");
	// buf.append(_ptr.type);
	// buf.append(" ");
	// buf.append(toString(_ptr.address));
	// buf.append(("]"));
	// return buf.toString();
	// }
	//
	// public static String toString(InformationProcessingTask _data) {
	// StringBuffer buf = new StringBuffer("[IPT: ");
	// buf.append(_data.id);
	// buf.append(" ");
	// buf.append(_data.taskName);
	// buf.append(("]"));
	// return buf.toString();
	// }

	// /**
	// * Get the canonical type name for a given class.
	// *
	// * @param <Type>
	// * @param _cls
	// * @return
	// */
	// public static final <Type> String typeName(Class<? extends Ice.Object>
	// _cls) {
	// return _cls.
	// }
	//

	/**
	 * Get the canonical type name for a given object.
	 * 
	 * @param <Type>
	 * @param _cls
	 * @return
	 */
	public static final <Type extends Ice.Object> String typeName(Type _obj) {
		return _obj.ice_id();
	}

	/**
	 * Get the canonical type name for a given class.
	 * 
	 * @param <Type>
	 * @param _cls
	 * @return
	 */
	public static final <Type extends Ice.Object> String typeName(
			Class<Type> _cls) {
		try {
			// TODO make so so so much more efficient
			return typeName(_cls.newInstance());
		} catch (Exception e) {
			throw new RuntimeException("can't derive is type from class", e);
		}
	}

	// public static final WorkingMemoryPermissions toPermissions(
	// WorkingMemoryLockRequest _request) {
	//
	// if (WorkingMemoryLockRequest.REQUEST_LOCK_O == _request
	// || WorkingMemoryLockRequest.REQUEST_TRY_LOCK_O == _request) {
	// return WorkingMemoryPermissions.LOCKEDO;
	// }
	// else if (WorkingMemoryLockRequest.REQUEST_LOCK_OD == _request
	// || WorkingMemoryLockRequest.REQUEST_TRY_LOCK_OD == _request) {
	// return WorkingMemoryPermissions.LOCKEDOD;
	// }
	// else if (WorkingMemoryLockRequest.REQUEST_LOCK_ODR == _request
	// || WorkingMemoryLockRequest.REQUEST_TRY_LOCK_ODR == _request) {
	// return WorkingMemoryPermissions.LOCKEDODR;
	// }
	// else {
	// return WorkingMemoryPermissions.UNLOCKED;
	// }
	//
	// }

	// public static final WorkingMemoryLockRequest toEnum(
	// WorkingMemoryPermissions _permissions, OperationMode _op) {
	// if (_permissions.equals(WorkingMemoryPermissions.LOCKEDO)) {
	// if (_op.equals(OperationMode.BLOCKING)) {
	// return WorkingMemoryLockRequest.REQUEST_LOCK_O;
	// }
	// else if (_op.equals(OperationMode.NON_BLOCKING)) {
	// return WorkingMemoryLockRequest.REQUEST_TRY_LOCK_O;
	// }
	// }
	// else if (_permissions.equals(WorkingMemoryPermissions.LOCKEDOD)) {
	// if (_op.equals(OperationMode.BLOCKING)) {
	// return WorkingMemoryLockRequest.REQUEST_LOCK_OD;
	// }
	// else if (_op.equals(OperationMode.NON_BLOCKING)) {
	// return WorkingMemoryLockRequest.REQUEST_TRY_LOCK_OD;
	// }
	// }
	// else if (_permissions.equals(WorkingMemoryPermissions.LOCKEDODR)) {
	// if (_op.equals(OperationMode.BLOCKING)) {
	// return WorkingMemoryLockRequest.REQUEST_LOCK_ODR;
	// }
	// else if (_op.equals(OperationMode.NON_BLOCKING)) {
	// return WorkingMemoryLockRequest.REQUEST_TRY_LOCK_ODR;
	// }
	// }
	//
	// throw new RuntimeException("invalid lock request");
	// }
	//
	// public static final String lockQueryString(String _id, String _subarch,
	// WorkingMemoryPermissions _permissions, OperationMode _op) {
	// StringBuffer buf = new StringBuffer();
	// buf.append(_id);
	// buf.append('\\');
	// buf.append(_subarch);
	// buf.append('\\');
	// buf.append(toEnum(_permissions, _op).value());
	// buf.append('\\');
	// return buf.toString();
	// }
	//
	// public static final String unlockQueryString(String _id, String _subarch)
	// {
	// StringBuffer buf = new StringBuffer();
	// buf.append(_id);
	// buf.append('\\');
	// buf.append(_subarch);
	// buf.append('\\');
	// buf.append(WorkingMemoryLockRequest.REQUEST_UNLOCK.value());
	// buf.append('\\');
	// return buf.toString();
	// }
	//
	// public static final String statusQueryString(String _id, String _subarch)
	// {
	// StringBuffer buf = new StringBuffer();
	// buf.append(_id);
	// buf.append('\\');
	// buf.append(_subarch);
	// buf.append('\\');
	// buf.append(WorkingMemoryLockRequest.REQUEST_STATUS.value());
	// buf.append('\\');
	// return buf.toString();
	// }
	//
	// public static String toString(WorkingMemoryPermissions _permissions) {
	// switch (_permissions.value()) {
	// case WorkingMemoryPermissions._LOCKEDO :
	// return "LOCKEDO";
	// case WorkingMemoryPermissions._LOCKEDOD :
	// return "LOCKEDOD";
	// case WorkingMemoryPermissions._LOCKEDODR :
	// return "LOCKEDODR";
	// case WorkingMemoryPermissions._UNLOCKED :
	// return "UNLOCKED";
	// case WorkingMemoryPermissions._DOES_NOT_EXIST :
	// return "DOES_NOT_EXIST";
	// }
	// throw new RuntimeException("invalid enum member: "
	// + _permissions.value());
	// }
	//
	/**
	 * Checks input enum for requisite permissions.
	 * 
	 * @param _permissions
	 * @return
	 */
	public final static boolean readAllowed(
			WorkingMemoryPermissions _permissions) {
		return _permissions == WorkingMemoryPermissions.UNLOCKED
				|| _permissions == WorkingMemoryPermissions.LOCKEDO
				|| _permissions == WorkingMemoryPermissions.LOCKEDOD;
	}

	/**
	 * Checks input enum for requisite permissions.
	 * 
	 * @param _permissions
	 * @return
	 */
	public final static boolean deleteAllowed(
			WorkingMemoryPermissions _permissions) {
		return _permissions == WorkingMemoryPermissions.UNLOCKED
				|| _permissions == WorkingMemoryPermissions.LOCKEDO;
	}

	/**
	 * Checks input enum for requisite permissions.
	 * 
	 * @param _permissions
	 * @return
	 */
	public final static boolean overwriteAllowed(
			WorkingMemoryPermissions _permissions) {
		return _permissions == WorkingMemoryPermissions.UNLOCKED;
	}

	public static Ice.Object createServant(Identity _id, ObjectAdapter _adapter)
			throws ClassNotFoundException, InstantiationException,
			IllegalAccessException {

		Class<?> componentClass = Class.forName(_id.category);

		// create the instance
		// Object process = (Object) constructor.newInstance(_id);
		CASTComponent component = (CASTComponent) componentClass.newInstance();
		component.setID(_id.name, new Current());
		component.setIceIdentity(_id);
		component.setObjectAdapter(_adapter);

		
		
		// TODO this won't work for generic components!

		// create the correct tie classes
		if (ManagedComponent.class.isAssignableFrom(componentClass)) {
			_ManagedComponentTie tie = new _ManagedComponentTie();
			tie.ice_delegate(component);
			return tie;
		} else if (UnmanagedComponent.class.isAssignableFrom(componentClass)) {
			_UnmanagedComponentTie tie = new _UnmanagedComponentTie();
			tie.ice_delegate(component);
			return tie;
		} else if (SubarchitectureWorkingMemory.class
				.isAssignableFrom(componentClass)) {
			_WorkingMemoryTie tie = new _WorkingMemoryTie();
			tie.ice_delegate(component);
			return tie;
		} else if (SubarchitectureTaskManager.class
				.isAssignableFrom(componentClass)) {
			_TaskManagerTie tie = new _TaskManagerTie();
			tie.ice_delegate(component);
			return tie;
		} else if (CASTComponentManager.class.isAssignableFrom(componentClass)) {
			_ComponentManagerTie tie = new _ComponentManagerTie();
			tie.ice_delegate(component);
			return tie;
		}
		// else if(Subar.class.isAssignableFrom(componentClass)) {
		// _UnmanagedComponentTie tie = new _UnmanagedComponentTie();
		// tie.ice_delegate(component);
		// return tie;
		// }
		// else if(UnmanagedComponent.class.isAssignableFrom(componentClass)) {
		// _UnmanagedComponentTie tie = new _UnmanagedComponentTie();
		// tie.ice_delegate(component);
		// return tie;
		// }
		else {
			return new _CASTComponentTie(component);
		}

	}

	public static String toString(CASTTime _time) {
		StringBuilder sb = new StringBuilder("[CASTTime: ");
		sb.append(_time.s);
		sb.append(":");
		sb.append(_time.us);
		sb.append("]");
		return sb.toString();
	}


	// public static String toString(WorkingMemoryLockRequest _request) {
	// switch (_request.value()) {
	// case WorkingMemoryLockRequest._REQUEST_LOCK_O :
	// return "LOCK_O";
	// case WorkingMemoryLockRequest._REQUEST_LOCK_OD :
	// return "LOCK_OD";
	// case WorkingMemoryLockRequest._REQUEST_LOCK_ODR :
	// return "LOCK_ODR";
	// case WorkingMemoryLockRequest._REQUEST_TRY_LOCK_O :
	// return "TRY_LOCK_O";
	// case WorkingMemoryLockRequest._REQUEST_TRY_LOCK_OD :
	// return "TRY_LOCK_OD";
	// case WorkingMemoryLockRequest._REQUEST_TRY_LOCK_ODR :
	// return "TRY_LOCK_ODR";
	// case WorkingMemoryLockRequest._REQUEST_UNLOCK :
	// return "UNLOCK";
	// case WorkingMemoryLockRequest._REQUEST_STATUS :
	// return "STATUS";
	// }
	//
	// throw new RuntimeException("invalid enum member: " + _request.value());
	// }
	//
	// public static Class<?> classFromType(String _type) {
	// return BALTType.classFromType(_type);
	// }
	//
	// public static String toString(TriBool _succeeded) {
	// switch (_succeeded.value()) {
	// case TriBool._triTrue :
	// return "triTrue";
	// case TriBool._triIndeterminate :
	// return "triIndeterminate";
	// case TriBool._triFalse :
	// return "triFalse";
	// }
	// throw new RuntimeException("invalid enum member: " + _succeeded.value());
	// }
	//
	// public static <Type> WorkingMemoryPointer workingMemoryPointer(String
	// _id, String _subarch, Class<Type> _type) {
	// return new WorkingMemoryPointer(typeName(_type), new
	// WorkingMemoryAddress(_id, _subarch));
	// }

}
