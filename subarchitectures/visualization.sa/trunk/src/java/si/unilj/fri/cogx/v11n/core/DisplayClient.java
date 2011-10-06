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
import Ice.ConnectionRefusedException;
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

		public void onDialogValueChanged(String dialogId, String name,
				String value, Ice.Current ctx) {
			if (m_Client != null)
				m_Client.onDialogValueChanged(dialogId, name, value);
		}
		public void handleDialogCommand(String dialogId, String name,
				String value, Ice.Current ctx) {
			if (m_Client != null)
				m_Client.handleDialogCommand(dialogId, name, value);
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
			if (m_standaloneHost != null) {
				m_standaloneHost = m_standaloneHost.trim();
				if (m_standaloneHost.length() == 0) m_standaloneHost = null;
				else if (m_standaloneHost == "/no") m_standaloneHost = null;
			}
		}
	}

	private final void connectToStandaloneHost(CASTComponent owner) {
		try {
			ObjectPrx prx = CASTUtils.getIceServer(
					V11NSTANDALONENAME.value,
					CASTUtils.toServantCategory(DisplayInterface.class),
					m_standaloneHost, V11NSTANDALONEPORT.value,
					owner.getObjectAdapter().getCommunicator());

			m_Server = DisplayInterfacePrxHelper.checkedCast(prx);
			owner.println("DisplayClient(java) connected to standalone server on '" + m_standaloneHost + "'.");

		}
		catch (Ice.ConnectionRefusedException e) {
			owner.println("*** Failed to connect to Standalone Display Server. Connection Refused.");
		}
	   	catch (Throwable t) {
			if (t.toString().indexOf("No description for:") >= 0) {
				owner.println("*** DisplayClient(java): DisplayServer not found.");
			}
			else owner.logException(t);
		}
	}

	/// Connect to the client to a display server.
	/// For the connection parameters and procedure see: CDisplayServer::m_standaloneHost (c++).
	public final void connectIceClient(CASTComponent owner) {
		assert(owner != null);
		m_Owner = owner;

		if (m_standaloneHost != null) {
			connectToStandaloneHost(owner);
		} else {
			try {
				m_Server = owner.getIceServer(m_ServerName,
						DisplayInterface.class, DisplayInterfacePrx.class);
			} catch (Throwable t) {
				m_Owner.logException(t);
				m_Server = null;
			}

			if (m_Server != null) {
				try {
					Ice.StringHolder hostname = new Ice.StringHolder();
					m_Server.getStandaloneHost(hostname);

					if (hostname.value == null || hostname.value.length() == 0) {
						owner.debug("DisplayClient(java) connected.");
					}
					else {
						m_standaloneHost = hostname.value;
						m_Server = null;
						owner.debug("DisplayClient(java) Redirecting connection to standalone display server.");
						connectToStandaloneHost(owner);
					}
				}
				catch (Ice.ConnectionRefusedException e) {
					owner.println("*** Failed to connect to Display Server. Connection Refused.");
				}
			   	catch (Throwable t) {
					m_Owner.logException(t);
				}
			}
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
		String myHost;
		try {
			myHost = m_Owner.getComponentManager().getComponentDescription(
					m_Owner.getComponentID()).hostName;
			m_Server.addClient(id, myHost, JAVASERVERPORT.value);
		} catch (CASTException e) {
			m_Owner.logException(e);
		}

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
		m_Server.setRawImage(id, image.width, image.height, 3, image.data);
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
		m_Server.setObjectPose3D(id, partId, position.x, position.y, position.z, rotation);
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

    public final void removeObject(String id)
	{
		if (m_Server == null)
			return;
		m_Server.removeObject(id);
	}
    public final void removePart(String id, String partId)
	{
		if (m_Server == null)
			return;
		m_Server.removePart(id, partId);
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

	public final void addAction(String viewId, ActionInfo info) {
		if (m_Server == null)
			return;
		Ice.Identity iceid = getEventClientId();
		m_Server.addAction(iceid, viewId, info);
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
	public void onDialogValueChanged(String dialogId, String name, String value) {
	}
	public void handleDialogCommand(String dialogId, String name, String value) {
	}

}
// vim:sw=4:ts=4:noet
