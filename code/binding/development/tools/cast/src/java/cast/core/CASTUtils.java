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

import java.util.ArrayList;
import java.util.Collection;

import balt.clients.ClientUtils;
import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import balt.corba.impl.FrameworkProcessManagerException;
import balt.core.data.BALTType;
import cast.cdl.FilterRestriction;
import cast.cdl.InformationProcessingTask;
import cast.cdl.OperationMode;
import cast.cdl.TaskDescription;
import cast.cdl.TaskDescriptionListHelper;
import cast.cdl.TaskGoal;
import cast.cdl.TaskManagementResult;
import cast.cdl.TaskOutcome;
import cast.cdl.TaskResult;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryAddressListHelper;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryEntryListHelper;
import cast.cdl.WorkingMemoryLockRequest;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.WorkingMemoryPointer;
import cast.cdl.guitypes.DrawBatch;
import cast.cdl.testing.CASTTestStruct;
import cast.configuration.ArchitectureConfigurationException;
import cast.configuration.CASTConnectionConfiguration;
import cast.core.data.CASTWorkingMemoryEntry;
import cast.core.interfaces.CWMELPullInterface.CWMELPullConnector;
import cast.core.interfaces.CWMELPushInterface.CWMELPushConnector;
import cast.core.interfaces.CWMEPullInterface.CWMEPullConnector;
import cast.core.interfaces.CWMEPushInterface.CWMEPushConnector;
import cast.core.interfaces.GMRPullInterface.GMRPullConnector;
import cast.core.interfaces.GMRPushInterface.GMRPushConnector;
import cast.core.interfaces.GPRPullInterface.GPRPullConnector;
import cast.core.interfaces.GPRPushInterface.GPRPushConnector;
import cast.core.interfaces.IPGPullInterface.IPGPullConnector;
import cast.core.interfaces.IPGPushInterface.IPGPushConnector;
import cast.core.interfaces.PermissionsPullInterface.PermissionsPullConnector;
import cast.core.interfaces.PermissionsPushInterface.PermissionsPushConnector;
import cast.core.interfaces.TDLPullInterface.TDLPullConnector;
import cast.core.interfaces.TDLPushInterface.TDLPushConnector;
import cast.core.interfaces.TGPullInterface.TGPullConnector;
import cast.core.interfaces.TGPushInterface.TGPushConnector;
import cast.core.interfaces.WMCFPullInterface.WMCFPullConnector;
import cast.core.interfaces.WMCFPushInterface.WMCFPushConnector;
import cast.core.interfaces.WMCPullInterface.WMCPullConnector;
import cast.core.interfaces.WMCPushInterface.WMCPushConnector;
import cast.core.interfaces.WMELPullInterface.WMELPullConnector;
import cast.core.interfaces.WMELPushInterface.WMELPushConnector;
import cast.core.interfaces.WMEPullInterface.WMEPullConnector;
import cast.core.interfaces.WMEPushInterface.WMEPushConnector;
import cast.ui.inspectable.GDBPullInterface.GDBPullConnector;
import cast.ui.inspectable.GDBPushInterface.GDBPushConnector;

/**
 * @author nah
 */
public class CASTUtils {

	/**
	 * @param _namingHost
	 * @param _millis
	 * @param connectionCollection
	 */
	private static void runCAST(String _namingHost, long _millis,
			Collection<ProcessConnection> connectionCollection) {
		ProcessConnection[] connections = new ProcessConnection[connectionCollection
				.size()];
		connectionCollection.toArray(connections);

		try {
			ClientUtils.runFramework(_namingHost, "1050", connections, _millis);
		}
		catch (FrameworkProcessManagerException e) {
			e.printStackTrace();
			System.err.println(e.getLocalizedMessage());

			System.exit(1);
		}
		catch (Throwable t) {
			t.printStackTrace();
			System.err.println(t.getLocalizedMessage());
			System.exit(1);
		}
	}

	public final static boolean equals(WorkingMemoryAddress _a,
			WorkingMemoryAddress _b) {
		if (_a == _b) {
			return true;
		}
		else {
			return _a.m_id.equals(_b.m_id)
					&& _a.m_subarchitecture.equals(_b.m_subarchitecture);
		}
	}

	
	
	
	/**
	 * Check equality for 2 working memory change structs.
	 * 
	 * @param _a
	 * @param _b
	 * @return
	 */
	public final static boolean equals(WorkingMemoryChange _a,
			WorkingMemoryChange _b) {
		if (_a == _b) {
			return true;
		}
		else {
			return equals(_a.m_address, _b.m_address)
					&& _a.m_operation == _b.m_operation
					&& _a.m_src.equals(_b.m_src) && _a.m_type.equals(_b.m_type);
		}
	}

	/**
	 * Check equality for 2 test structs.
	 * 
	 * @param _a
	 * @param _b
	 * @return
	 */
	public final static boolean equals(CASTTestStruct _a, CASTTestStruct _b) {
		if (_a == _b) {
			return true;
		}
		else {
			return equals(_a.m_change, _b.m_change) && _a.m_count == _b.m_count;
		}
	}

	/**
	 * 
	 */
	public static void initialiseCAST() {
		// System.out.println("initialiseCAST");

		// Code to add data types to framework
		// ClientUtils.addObjectDatatype("WorkingMemoryEntry",
		// WorkingMemoryEntry.class, WMEPushConnector.class,
		// WMEPullConnector.class);

		// if
		// (!LocalConnectionFactory.supportsClass("WorkingMemoryEntry"))
		// {
		// // System.out.println("Adding " + _dataType
		// // + " datatype to balt.");
		// // registers datatype with connection factory
		// LocalConnectionFactory.addDatatype("WorkingMemoryEntry",
		// WorkingMemoryEntry.class, WMEPushConnector.class,
		// WMEPullConnector.class);
		// // registers datatype with translator for cross
		// // language/machine support
		// RemoteDataTranslator.addTranslators(new WMETranslator());
		// }
		//
		// if (!LocalConnectionFactory
		// .supportsClass("WorkingMemoryEntryList")) {
		// // System.out.println("Adding " + _dataType
		// // + " datatype to balt.");
		// // registers datatype with connection factory
		// LocalConnectionFactory.addDatatype(
		// "WorkingMemoryEntryList", WorkingMemoryEntry[].class,
		// WMELPushConnector.class, WMELPullConnector.class);
		// // registers datatype with translator for cross
		// // language/machine support
		// RemoteDataTranslator.addTranslators(new WMELTranslator());
		// }

		ClientUtils.addObjectDatatype(WorkingMemoryEntry.class,
				WMEPushConnector.class, WMEPullConnector.class);

		ClientUtils.addSequenceDatatype(WorkingMemoryEntry[].class,
				WorkingMemoryEntryListHelper.class, WMELPushConnector.class,
				WMELPullConnector.class);

		ClientUtils.addLocalObjectDatatype(CASTWorkingMemoryEntry.class,
				CWMEPushConnector.class, CWMEPullConnector.class);

		ClientUtils.addObjectDatatype(WorkingMemoryChange.class,
				WMCPushConnector.class, WMCPullConnector.class);

		ClientUtils.addObjectDatatype(WorkingMemoryChangeFilter.class,
				WMCFPushConnector.class, WMCFPullConnector.class);

		ClientUtils.addLocalObjectDatatype(CASTWorkingMemoryEntry[].class,
				CWMELPushConnector.class, CWMELPullConnector.class);

		// ClientUtils.addSequenceDatatype("WorkingMemoryChangeList",
		// WorkingMemoryChange[].class,
		// WorkingMemoryChangeListHelper.class, WMCLPushConnector.class,
		// WMCLPullConnector.class);

		ClientUtils.addObjectDatatype(InformationProcessingTask.class,
				IPGPushConnector.class, IPGPullConnector.class);

		ClientUtils.addObjectDatatype(TaskManagementResult.class,
				GMRPushConnector.class, GMRPullConnector.class);

		ClientUtils.addObjectDatatype(TaskResult.class, GPRPushConnector.class,
				GPRPullConnector.class);

		ClientUtils.addObjectDatatype(TaskGoal.class, TGPushConnector.class,
				TGPullConnector.class);

		ClientUtils.addSequenceDatatype(TaskDescription[].class,
				TaskDescriptionListHelper.class, TDLPushConnector.class,
				TDLPullConnector.class);

		ClientUtils.addObjectDatatype(DrawBatch.class, GDBPushConnector.class,
				GDBPullConnector.class);

		CASTDatatypeManager.addObjectDatatype(WorkingMemoryAddress.class);

		CASTDatatypeManager.addSequenceDatatype(WorkingMemoryAddress[].class,
				WorkingMemoryAddressListHelper.class);

		// new additions for locking
		ClientUtils.addObjectDatatype(WorkingMemoryPermissions.class,
				PermissionsPushConnector.class, PermissionsPullConnector.class);

	}

	public static void runCAST(String _namingHost,
			CASTConnectionConfiguration _arch, long _millis)
			throws ArchitectureConfigurationException {
		runCAST(_namingHost, _millis, _arch.getConnections());
	}

	public static void runCAST(String _namingHost,
			CASTConnectionConfiguration _arch, ProcessConnection[] _extras,
			long _millis) throws ArchitectureConfigurationException {

		ArrayList<ProcessConnection> connections = _arch.getConnections();
		for (int i = 0; i < _extras.length; i++) {
			connections.add(_extras[i]);
		}

		// for (ProcessConnection connection : connections) {
		// System.out.println(connection.m_connectionID);
		// }

		runCAST(_namingHost, _millis, connections);
	}

	/**
	 * @param _outcome
	 * @return
	 */
	public static String toString(TaskOutcome _outcome) {
		switch (_outcome.value()) {
			case TaskOutcome._PROCESSING_COMPLETE :
				return "complete";
			case TaskOutcome._PROCESSING_COMPLETE_FAILURE :
				return "failure";
			case TaskOutcome._PROCESSING_COMPLETE_SUCCESS :
				return "success";
			case TaskOutcome._PROCESSING_INCOMPLETE :
				return "incomplete";
		}
		return "unknown value";
	}

	/**
	 * @param _wma
	 * @return
	 */
	public static String toString(WorkingMemoryAddress _wma) {
		StringBuffer buf = new StringBuffer("[WMA id = ");
		buf.append(_wma.m_id);
		buf.append(" : sa = ");
		buf.append(_wma.m_subarchitecture);
		buf.append("]");
		return buf.toString();
	}

	/**
	 * @param _wma
	 * @return
	 */
	public static String toString(WorkingMemoryChange _wmc) {
		StringBuffer buf = new StringBuffer("[WMC ");
		buf.append(_wmc.m_src);
		buf.append(" ");
		buf.append(_wmc.m_type);
		buf.append(" ");
		buf.append(toString(_wmc.m_address));
		buf.append(" ");
		buf.append(toString(_wmc.m_operation));
		buf.append("]");
		return buf.toString();
	}

	/**
	 * @param _filter
	 * @return
	 */
	public static String toString(WorkingMemoryChangeFilter _filter) {
		StringBuffer buf = new StringBuffer("[WMCF: ");
		buf.append(toString(_filter.m_operation));
		buf.append(" ");
		buf.append(_filter.m_src);
		buf.append(" ");
		buf.append(toString(_filter.m_address));
		buf.append(" ");
		buf.append(_filter.m_type);
		buf.append(" ");
		buf.append(toString(_filter.m_restriction));
		buf.append(" (");
		buf.append(_filter.m_origin);
		buf.append(")]");
		return buf.toString();
	}

	public static String toString(FilterRestriction _restriction) {
		switch (_restriction.value()) {
			case FilterRestriction._ALL_SA :
				return "ALL_SA";
			case FilterRestriction._LOCAL_SA :
				return "LOCAL_SA";
		}
		throw new RuntimeException("Unknown enum value: "
				+ _restriction.value());

	}

	public static String toString(WorkingMemoryOperation _op) {
		switch (_op.value()) {
			case WorkingMemoryOperation._ADD :
				return "ADD";
			case WorkingMemoryOperation._OVERWRITE :
				return "OVERWRITE";
			case WorkingMemoryOperation._DELETE :
				return "DELETE";
			case WorkingMemoryOperation._GET :
				return "GET";
			default :
				return "UNKNOWN";
		}
	}

	/**
	 * @param _ptr
	 * @return
	 */
	public static String toString(WorkingMemoryPointer _ptr) {
		StringBuffer buf = new StringBuffer("[WMP: ");
		buf.append(_ptr.m_type);
		buf.append(" ");
		buf.append(toString(_ptr.m_address));
		buf.append(("]"));
		return buf.toString();
	}

	public static String toString(InformationProcessingTask _data) {
		StringBuffer buf = new StringBuffer("[IPT: ");
		buf.append(_data.m_id);
		buf.append(" ");
		buf.append(_data.m_taskName);
		buf.append(("]"));
		return buf.toString();
	}

	/**
	 * Get the canonical type name for a given class.
	 * 
	 * @param <Type>
	 * @param _cls
	 * @return
	 */
	public static final <Type> String typeName(Class<Type> _cls) {
		return BALTType.typeName(_cls);
	}

	/**
	 * Get the canonical type name for a given object.
	 * 
	 * @param <Type>
	 * @param _cls
	 * @return
	 */
	public static final <Type> String typeName(Type _obj) {
		return BALTType.typeName(_obj.getClass());
	}

	public static final WorkingMemoryPermissions toPermissions(
			WorkingMemoryLockRequest _request) {

		if (WorkingMemoryLockRequest.REQUEST_LOCK_O == _request
				|| WorkingMemoryLockRequest.REQUEST_TRY_LOCK_O == _request) {
			return WorkingMemoryPermissions.LOCKED_O;
		}
		else if (WorkingMemoryLockRequest.REQUEST_LOCK_OD == _request
				|| WorkingMemoryLockRequest.REQUEST_TRY_LOCK_OD == _request) {
			return WorkingMemoryPermissions.LOCKED_OD;
		}
		else if (WorkingMemoryLockRequest.REQUEST_LOCK_ODR == _request
				|| WorkingMemoryLockRequest.REQUEST_TRY_LOCK_ODR == _request) {
			return WorkingMemoryPermissions.LOCKED_ODR;
		}
		else {
			return WorkingMemoryPermissions.UNLOCKED;
		}

	}

	public static final WorkingMemoryLockRequest toEnum(
			WorkingMemoryPermissions _permissions, OperationMode _op) {
		if (_permissions.equals(WorkingMemoryPermissions.LOCKED_O)) {
			if (_op.equals(OperationMode.BLOCKING)) {
				return WorkingMemoryLockRequest.REQUEST_LOCK_O;
			}
			else if (_op.equals(OperationMode.NON_BLOCKING)) {
				return WorkingMemoryLockRequest.REQUEST_TRY_LOCK_O;
			}
		}
		else if (_permissions.equals(WorkingMemoryPermissions.LOCKED_OD)) {
			if (_op.equals(OperationMode.BLOCKING)) {
				return WorkingMemoryLockRequest.REQUEST_LOCK_OD;
			}
			else if (_op.equals(OperationMode.NON_BLOCKING)) {
				return WorkingMemoryLockRequest.REQUEST_TRY_LOCK_OD;
			}
		}
		else if (_permissions.equals(WorkingMemoryPermissions.LOCKED_ODR)) {
			if (_op.equals(OperationMode.BLOCKING)) {
				return WorkingMemoryLockRequest.REQUEST_LOCK_ODR;
			}
			else if (_op.equals(OperationMode.NON_BLOCKING)) {
				return WorkingMemoryLockRequest.REQUEST_TRY_LOCK_ODR;
			}
		}

		throw new RuntimeException("invalid lock request");
	}

	public static final String lockQueryString(String _id, String _subarch,
			WorkingMemoryPermissions _permissions, OperationMode _op) {
		StringBuffer buf = new StringBuffer();
		buf.append(_id);
		buf.append('\\');
		buf.append(_subarch);
		buf.append('\\');
		buf.append(toEnum(_permissions, _op).value());
		buf.append('\\');
		return buf.toString();
	}

	public static final String unlockQueryString(String _id, String _subarch) {
		StringBuffer buf = new StringBuffer();
		buf.append(_id);
		buf.append('\\');
		buf.append(_subarch);
		buf.append('\\');
		buf.append(WorkingMemoryLockRequest.REQUEST_UNLOCK.value());
		buf.append('\\');
		return buf.toString();
	}

	public static final String statusQueryString(String _id, String _subarch) {
		StringBuffer buf = new StringBuffer();
		buf.append(_id);
		buf.append('\\');
		buf.append(_subarch);
		buf.append('\\');
		buf.append(WorkingMemoryLockRequest.REQUEST_STATUS.value());
		buf.append('\\');
		return buf.toString();
	}

	public static String toString(WorkingMemoryPermissions _permissions) {
		switch (_permissions.value()) {
			case WorkingMemoryPermissions._LOCKED_O :
				return "LOCKED_O";
			case WorkingMemoryPermissions._LOCKED_OD :
				return "LOCKED_OD";
			case WorkingMemoryPermissions._LOCKED_ODR :
				return "LOCKED_ODR";
			case WorkingMemoryPermissions._UNLOCKED :
				return "UNLOCKED";
			case WorkingMemoryPermissions._DOES_NOT_EXIST :
				return "DOES_NOT_EXIST";
		}
		throw new RuntimeException("invalid enum member: "
				+ _permissions.value());
	}

	/**
	 * Checks input enum for requisite permissions.
	 * 
	 * @param _permissions
	 * @return
	 */
	public final static boolean readAllowed(
			WorkingMemoryPermissions _permissions) {
		return _permissions == WorkingMemoryPermissions.UNLOCKED
				|| _permissions == WorkingMemoryPermissions.LOCKED_O
				|| _permissions == WorkingMemoryPermissions.LOCKED_OD;
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
				|| _permissions == WorkingMemoryPermissions.LOCKED_O;
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

	public static String toString(WorkingMemoryLockRequest _request) {
		switch (_request.value()) {
			case WorkingMemoryLockRequest._REQUEST_LOCK_O :
				return "LOCK_O";
			case WorkingMemoryLockRequest._REQUEST_LOCK_OD :
				return "LOCK_OD";
			case WorkingMemoryLockRequest._REQUEST_LOCK_ODR :
				return "LOCK_ODR";
			case WorkingMemoryLockRequest._REQUEST_TRY_LOCK_O :
				return "TRY_LOCK_O";
			case WorkingMemoryLockRequest._REQUEST_TRY_LOCK_OD :
				return "TRY_LOCK_OD";
			case WorkingMemoryLockRequest._REQUEST_TRY_LOCK_ODR :
				return "TRY_LOCK_ODR";
			case WorkingMemoryLockRequest._REQUEST_UNLOCK :
				return "UNLOCK";
			case WorkingMemoryLockRequest._REQUEST_STATUS :
				return "STATUS";
		}

		throw new RuntimeException("invalid enum member: " + _request.value());
	}

	public static Class<?> classFromType(String _type) {
		return BALTType.classFromType(_type);
	}

	public static String toString(TriBool _succeeded) {
		switch (_succeeded.value()) {
			case TriBool._triTrue :
				return "triTrue";
			case TriBool._triIndeterminate :
				return "triIndeterminate";
			case TriBool._triFalse :
				return "triFalse";
		}
		throw new RuntimeException("invalid enum member: " + _succeeded.value());
	}

	public static <Type>  WorkingMemoryPointer workingMemoryPointer(String _id, String _subarch, Class<Type> _type) {
		return new WorkingMemoryPointer(typeName(_type), new WorkingMemoryAddress(_id, _subarch));
	}

}
