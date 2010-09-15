/*
 * author: Marko Mahnič
 * created: 2010-07-11
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
package si.unilj.fri.cogx.v11n.core;

//-----------------------------------------------------------------
// CAST IMPORTS
//-----------------------------------------------------------------
import java.util.Map;

import Ice.ObjectPrx;
import Visualization.ActionInfo;
import Visualization.DisplayInterface;
import Visualization.DisplayInterfacePrx;
import Visualization.DisplayInterfacePrxHelper;
import Visualization.Quaternion;
import Visualization.TFormFieldMapHolder;
import Visualization.V11NSTANDALONENAME;
import Visualization.V11NSTANDALONEPORT;
import cast.CASTException;
import cast.cdl.JAVASERVERPORT;
import cast.core.CASTComponent;
import cast.core.CASTUtils;

public class DisplayClient {
	private class EventReceiverImpl extends Visualization._EventReceiverDisp {
		/**
	 * 
	 */
		private static final long serialVersionUID = -8403217494649616812L;
		private DisplayClient m_Client;

		public EventReceiverImpl(DisplayClient client) {
			m_Client = client;
		}

		public void handleEvent(Visualization.TEvent event, Ice.Current ctx) {
			if (m_Client != null)
				m_Client.handleEvent(event);
		}

		public String getControlState(String ctrlId, Ice.Current ctx) {
			if (m_Client != null)
				return m_Client.getControlState(ctrlId);
			return "";
		}

		public void handleForm(String id, String partId,
				Map<String, String> fields, Ice.Current ctx) {
			if (m_Client != null)
				m_Client.handleForm(id, partId, fields);
		}

		public boolean getFormData(String id, String partId,
				TFormFieldMapHolder fields, Ice.Current ctx) {
			if (m_Client != null)
				return m_Client.getFormData(id, partId, fields);
			return false;
		}
	}

	private String m_ServerName = "display.srv";
	private String m_standaloneHost;

	private DisplayInterfacePrx m_Server = null;
	private CASTComponent m_Owner = null;
	private EventReceiverImpl m_EventReceiver = null;

	public final void configureDisplayClient(
			java.util.Map<String, String> config) {
		if (config.containsKey("--displayserver")) {
			m_ServerName = config.get("--displayserver");
		}
		if (config.containsKey("--standalone-display-host")) {
			m_standaloneHost = config.get("--standalone-display-host").trim();
			if (m_standaloneHost != null && m_standaloneHost.trim().length() == 0) {
				m_standaloneHost = null;
			}
		}
	}

	public final void connectIceClient(CASTComponent owner) {
		m_Owner = owner;
		try {

			if (m_standaloneHost != null) {

				ObjectPrx prx = CASTUtils.getIceServer(
						V11NSTANDALONENAME.value, CASTUtils
								.toServantCategory(DisplayInterface.class),
						m_standaloneHost, V11NSTANDALONEPORT.value, m_Owner
								.getObjectAdapter().getCommunicator());

				m_Server = DisplayInterfacePrxHelper.checkedCast(prx);
				m_Owner.println("DisplayClient(java) connected to standalone server.");

			} else {
				m_Server = owner.getIceServer(m_ServerName,
						DisplayInterface.class, DisplayInterfacePrx.class);

			}
		} catch (Throwable t) {
			if (t.toString().indexOf("No description for:") >= 0) {
				m_Owner.println("*** DisplayClient(java): DisplayServer not found.");
			}
			else m_Owner.logException(t);
		}
	}

	private String getComponentId() {
		if (m_Owner == null)
			return "[null]";
		return m_Owner.getComponentID();
	}

	private Ice.Identity getEventClientId() {
		Ice.Identity id = new Ice.Identity();
		id.name = getComponentId();
		id.category = "Visualization.EventReceiver";
		return id;
	}

	public final void installEventReceiver() // throws(std::runtime_error)
	{
		if (m_Owner == null) {
			// TODO: throw std::runtime_error(cast::exceptionMessage(__HERE__,
			// "CDisplayClient: connectIceClient() must be called before installEventReciever()."));
			System.out.println(" *** DisplayClient(java): Owner is null");
			return;
		}

		if (m_Server == null) {
			// TODO: log("CActiveDisplayClient: server not connected.");
			m_Owner.println(" *** DisplayClient(java): Server is null (not connected)");
			return;
		}

		if (m_EventReceiver != null) {
			// TODO: log("CActiveDisplayClient already has an EventReceiver.");
			m_Owner.println(" *** DisplayClient(java): EventReceiver already installed");
			return;
		}

		Ice.Identity id = getEventClientId();
		m_EventReceiver = new EventReceiverImpl(this);
		m_Owner.registerIceServer(Visualization.EventReceiver.class,
				m_EventReceiver);
		String myHost = m_Owner.getComponentManager().getComponentDescription(
				m_Owner.getComponentID()).hostName;
		m_Server.addClient(id, myHost, JAVASERVERPORT.value);
	}

	public final void setImage(String id, int width, int height, int channels,
			byte data[]) {
		if (m_Server == null)
			return;
		m_Server.setRawImage(id, width, height, channels, data);
	}

	public final void setImage(String id, Video.Image image) {
		if (m_Server == null)
			return;
		m_Server.setImage(id, image);
	}

	public final void setCompressedImage(String id, byte[] data, String format) {
		if (m_Server == null)
			return;
		m_Server.setCompressedImage(id, data, format);
	}

	public final void setCompressedImage(String id, byte[] data) {
		if (m_Server == null)
			return;
		m_Server.setCompressedImage(id, data, "");
	}

	public final void setObject(String id, String partId, String svgObject) {
		if (m_Server == null)
			return;
		m_Server.setObject(id, partId, svgObject);
	}

	public final void setLuaGlObject(String id, String partId, String script) {
		if (m_Server == null)
			return;
		m_Server.setLuaGlObject(id, partId, script);
	}

	public final void setObjectPose3D(String id, String partId,
			cogx.Math.Vector3 position, Quaternion rotation) {
		if (m_Server == null)
			return;
		m_Server.setObjectPose3D(id, partId, position, rotation);
	}

	public final void setObjectTransform2D(String id, String partId,
			double[] matrix33) {
		if (m_Server == null)
			return;
		m_Server.setObjectTransform2D(id, partId, matrix33);
	}

	public final void setHtml(String id, String partId, String htmlData) {
		if (m_Server == null)
			return;
		m_Server.setHtml(id, partId, htmlData);
	}

	public final void setHtmlHead(String id, String partId, String htmlData) {
		if (m_Server == null)
			return;
		m_Server.setHtmlHead(id, partId, htmlData);
	}

	public final void setActiveHtml(String id, String partId, String htmlData) {
		if (m_Server == null)
			return;
		Ice.Identity iceid = getEventClientId();
		m_Server.setActiveHtml(iceid, id, partId, htmlData);
	}

	public final void setHtmlForm(String id, String partId, String htmlData) {
		if (m_Server == null)
			return;
		Ice.Identity iceid = getEventClientId();
		m_Server.setHtmlForm(iceid, id, partId, htmlData);
	}

	public final void setHtmlFormData(String id, String partId,
			Map<String, String> fields) {
		if (m_Server == null)
			return;
		m_Server.setHtmlFormData(id, partId, fields);
	}

	public final void addButton(String viewId, String ctrlId, String label) {
		if (m_Server == null)
			return;
		Ice.Identity iceid = getEventClientId();
		m_Server.addButton(iceid, viewId, ctrlId, label);
	}

	public final void addCheckBox(String viewId, String ctrlId, String label) {
		if (m_Server == null)
			return;
		Ice.Identity iceid = getEventClientId();
		m_Server.addCheckBox(iceid, viewId, ctrlId, label);
	}

	public final void addToolButton(String viewId, String ctrlId,
			ActionInfo info) {
		if (m_Server == null)
			return;
		Ice.Identity iceid = getEventClientId();
		m_Server.addToolButton(iceid, viewId, ctrlId, info);
	}

	// public final void enableMouseEvents(String viewId, boolean enabled)
	// {
	// if (m_Server == null) return;
	// Ice.Identity iceid = getEventClientId();
	// m_Server.enableMouseEvents(viewId, enabled);
	// }

	// -----------------------------------------------------------------
	// Event Receiver Methods - to be overridden
	// -----------------------------------------------------------------
	public void handleEvent(Visualization.TEvent event) {
	}

	public String getControlState(String ctrlId) {
		return "";
	}

	public void handleForm(String id, String partId, Map<String, String> fields) {
	}

	public boolean getFormData(String id, String partId,
			TFormFieldMapHolder fields) {
		return false;
	}

}
// vim:sw=4:ts=4:noet
