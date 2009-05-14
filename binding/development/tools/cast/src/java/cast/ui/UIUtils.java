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
package cast.ui;

import balt.clients.ClientUtils;
import balt.jni.NativeProcessLauncher;
import cast.cdl.guitypes.*;
import cast.cdl.ui.*;
import cast.ui.architecture.interfaces.ComponentEventPullInterface.ComponentEventPullConnector;
import cast.ui.architecture.interfaces.ComponentEventPushInterface.ComponentEventPushConnector;
import cast.ui.architecture.interfaces.ComponentStatusPullInterface.ComponentStatusPullConnector;
import cast.ui.architecture.interfaces.ComponentStatusPushInterface.ComponentStatusPushConnector;
import cast.ui.architecture.interfaces.TextOutputPullInterface.TextOutputPullConnector;
import cast.ui.architecture.interfaces.TextOutputPushInterface.TextOutputPushConnector;
import cast.ui.inspectable.GDBPullInterface.GDBPullConnector;
import cast.ui.inspectable.GDBPushInterface.GDBPushConnector;

/**
 * @author nah
 */
public class UIUtils {

	private static boolean m_bInit = false;

	public static void init() {
		if (!m_bInit) {

			ClientUtils.addObjectDatatype(DrawBatch.class,
					GDBPushConnector.class, GDBPullConnector.class);

			ClientUtils.addObjectDatatype(TextOutput.class,
					TextOutputPushConnector.class,
					TextOutputPullConnector.class);

			ClientUtils.addObjectDatatype(ComponentStatus.class,
					ComponentStatusPushConnector.class,
					ComponentStatusPullConnector.class);

			ClientUtils.addObjectDatatype(ComponentEvent.class,
					ComponentEventPushConnector.class,
					ComponentEventPullConnector.class);
			m_bInit = true;
		}
	}

	public static ComponentStatus newComponentStatus() {
		return new ComponentStatus("", "", false, false, false, false, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0);
	}

	public static boolean changed(ComponentStatus _before,
			ComponentStatus _after) {
		return _before.m_locked != _after.m_locked
				|| _before.m_sleeping != _after.m_sleeping
				|| _before.m_changeQueue != _after.m_changeQueue
				|| _before.m_totalChangeEventsFiltered != _after.m_totalChangeEventsFiltered
				|| _before.m_totalChangeEventsReceived != _after.m_totalChangeEventsReceived;
	}

	public static boolean equals(ComponentStatus _cs1, ComponentStatus _cs2) {
		return _cs1.m_debug == _cs2.m_debug
				&& _cs1.m_locked == _cs2.m_locked
				&& _cs1.m_log == _cs2.m_log
				&& _cs1.m_sleeping == _cs2.m_sleeping
				&& _cs1.m_changeQueue == _cs2.m_changeQueue
				&& _cs1.m_component.equals(_cs2.m_component)
				&& _cs1.m_subarchitecture.equals(_cs2.m_subarchitecture)
				&& _cs1.m_totalChangeEventsFiltered == _cs2.m_totalChangeEventsFiltered
				&& _cs1.m_totalChangeEventsReceived == _cs2.m_totalChangeEventsReceived;
	}

	//    
	// public static void logMemoryWrite(WorkingMemoryOperation _op,
	// ComponentStatus _cs) {
	// synchronized (_cs) {
	// switch (_op.value()) {
	// case WorkingMemoryOperation._ADD:
	// _cs.m_totalAdds++;
	// break;
	// case WorkingMemoryOperation._OVERWRITE:
	// _cs.m_totalOverwrites++;
	// break;
	// case WorkingMemoryOperation._DELETE:
	// _cs.m_totalDeletes++;
	// break;
	// default:
	// break;
	// }
	// }
	// }

	public static String toString(ComponentEventType _type) {
		switch (_type.value()) {
		case ComponentEventType._ADD:
			return "ADD";
		case ComponentEventType._OVERWRITE:
			return "OVR";
		case ComponentEventType._DELETE:
			return "DEL";
		case ComponentEventType._PROPOSED:
			return "PRO";
		case ComponentEventType._START:
			return "STA";
		case ComponentEventType._END:
			return "END";
		case ComponentEventType._GET:
			return "GET";
		default:
			return "UNK";
		}
	}

	public static String toString(ComponentEvent _event) {
		StringBuffer sb = new StringBuffer("ComponentEvent ");
		sb.append(_event.m_componentID);
		sb.append(": ");
		sb.append(toString(_event.m_event));
		sb.append(" ");
		sb.append(_event.m_target);
		sb.append(" ");
		sb.append(_event.m_dataType);
		sb.append(" ");
		sb.append(_event.m_dataID);
		sb.append(" ");
		sb.append(NativeProcessLauncher.toString(_event.m_time));
		return sb.toString();
	}

	public static String toString(ComponentStatus _cs) {
		StringBuffer sb = new StringBuffer("ComponentStatus ");
		sb.append(_cs.m_component);
		sb.append(": ");
		// sb.append(_cs.m_locked);
		// sb.append(" ");
		// sb.append(_cs.m_sleeping);
		sb.append(_cs.m_changeQueue);
		sb.append("/");
		sb.append(_cs.m_totalChangeEventsFiltered);
		sb.append("/");
		sb.append(_cs.m_totalChangeEventsReceived);
		return sb.toString();
	}
}