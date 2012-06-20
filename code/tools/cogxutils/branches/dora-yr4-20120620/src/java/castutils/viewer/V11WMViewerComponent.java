/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package castutils.viewer;

import java.util.HashMap;
import java.util.Map;
import java.util.StringTokenizer;
import java.util.Vector;

import si.unilj.fri.cogx.v11n.core.DisplayClient;
import Ice.ObjectImpl;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.WMEntrySet;
import castutils.castextensions.WMEntrySet.ChangeHandler;
import castutils.viewer.plugins.DefaultXMLInfo;
import castutils.viewer.plugins.Plugin;
import castutils.castextensions.IceXMLSerializer;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class V11WMViewerComponent extends ManagedComponent {

	final WMEntrySet entrySet;
	final private MyDisplayClient displayClient = new MyDisplayClient();
	final Map<Class<?>, Plugin> objectDispatcherMap = new HashMap<Class<?>, Plugin>();
	public boolean addGenericCol = false;
	public boolean logGenericCol = false;
	public boolean compactGenericCol = false;
	public String omittedFields = null;
	private boolean foundLargeGeneric = false;
	private final DefaultXMLInfo genericPlugin = new DefaultXMLInfo();

	private class MyDisplayClient extends DisplayClient implements
			ChangeHandler {

		private String v11nObject;

		@Override
		public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
				WorkingMemoryChange wmc, ObjectImpl newEntry,
				ObjectImpl oldEntry) throws CASTException {
			try {
				String v11part = "020_" + addrToString(wmc.address);
				switch (wmc.operation) {
				case ADD:
				case OVERWRITE:
					Plugin pluginToCall = findPlugin(newEntry.getClass());

					Vector<Object> row = new Vector<Object>();
					// mark additions
					if (wmc.operation == WorkingMemoryOperation.ADD)
						row.add("*");
					else
						row.add("");
					row.add(addrToString(wmc.address));
					row.add(newEntry.getClass().getSimpleName());
					if (pluginToCall != null) { // if we have a plugin for this
						// object
						Vector<Object> extraInfo = pluginToCall
								.toVector(newEntry);
						row.addAll(extraInfo);
					}
					String logString = "";
					for (Object o : row) {
						logString += "<td>" + o.toString() + "</td>";
					}
					if (addGenericCol) {
					  if (omittedFields != null) {
							// @author: mmarko
							// Don't serialize the fileds listed in omittedFields.
							StringTokenizer st = new StringTokenizer(omittedFields);
							while (st.hasMoreTokens()) {
								String fieldName = st.nextToken();
								IceXMLSerializer.omitField(newEntry.getClass(), fieldName);
							}
							omittedFields = null;
					  }
						String genericText = (String) genericPlugin.toVector(
								newEntry).get(0);
						if (genericText.length() < 500)
						  logString += "<td>" + genericText + "</td>";
						else {
							logString += "<td class='largeinfo'><div class='top'>" + genericText + "</div></td>";
							if (!foundLargeGeneric) {
								foundLargeGeneric = true;
								if (addGenericCol && !compactGenericCol) {
									setHtml(v11nObject, "999_info",
											"(use the --compact option to reduce the size of the generic column; use --omit-fields to hide some fields)");
								}
							}
						}
						if (logGenericCol)
						  getLogger().debug(CASTUtils.toString(wmc) + genericText);
					}
					setHtml(v11nObject, v11part, "<tr>" + logString + "</tr>");
					break;
				case DELETE:
					removePart(v11nObject, v11part);
					break;
				}

			} catch (Exception e) {
				getLogger()
						.warn(
								"there was an exception in the viewer but we happily ignore that for now: ",
								e);
			}
		}

		private String addrToString(WorkingMemoryAddress wma) {
			return wma.id + "@" + wma.subarchitecture;
		}

		private void initDisplay() {
 			v11nObject = "wm." + getComponentID();

			if (compactGenericCol) {
				String style = "<style>"
					+ "td.largeinfo { height: 20em; }"
					+ "td.largeinfo div.top { height: 100%; overflow: auto; background: lightgray; font-size: 90%; }"
					+ "</style>";
				setHtmlHead(v11nObject, "000_table-style", style);
			}

			String tableHdr = "<table frame=\"border\" border=\"1\" rules=\"all\">"
				+ "<tr><th>NEW?</th><th>address</th><th>type</th>"
				+ "<th>info1</th><th>info2</th><th>info3</th><th>info4</th></tr>";

			setHtml(v11nObject, "010_table-header", tableHdr);
			setHtml(v11nObject, "100_table-end", "</table>");

		}
	}

	/**
	 * 
	 */
	public V11WMViewerComponent() {
		entrySet = WMEntrySet.create(this);
		entrySet.setHandler(displayClient);
	}

	public Plugin findPlugin(Class<? extends ObjectImpl> origType) {
		Class<?> oType = origType;
		if (objectDispatcherMap.containsKey(oType))
			return objectDispatcherMap.get(oType);
		// if not yet found, look for supertype plugins
		Plugin pluginToCall = null;
		while (pluginToCall == null) {
			String SimpleName = oType.getSimpleName();
			String fullName = this.getClass().getPackage().getName()
					+ ".plugins." + SimpleName + "Info";
			try {
				ClassLoader.getSystemClassLoader().loadClass(fullName);
				pluginToCall = (Plugin) Class.forName(fullName).newInstance();
				break;
			} catch (ClassNotFoundException e) {
			} catch (InstantiationException e) {
			} catch (IllegalAccessException e) {
			}
			oType = oType.getSuperclass();
			if (oType == null) // if no superclass exists, we have to give up
				break;
			if (oType.equals(Ice.Object.class)) // we don't need to look up
				// further
				break;

		}
		objectDispatcherMap.put(origType, pluginToCall);
		return pluginToCall;
	}

	@SuppressWarnings("unchecked")
	@Override
	protected void configure(Map<String, String> arg0) {
		String subscrStr = arg0.get("--subscribe");
		if (subscrStr != null) {
			StringTokenizer st = new StringTokenizer(subscrStr, ",");
			while (st.hasMoreTokens()) {
				String className = st.nextToken();
				className = className.trim();
				try {
					System.out.println("add type '" + className + "'");
					ClassLoader.getSystemClassLoader().loadClass(className);
					entrySet.addType((Class<? extends ObjectImpl>) Class
							.forName(className));
				} catch (ClassNotFoundException e) {
					println("trying to register for a class that doesn't exist: "
							+ className);
				}
			}
		}
		if (arg0.get("--generic-col") != null) {
			addGenericCol = true;
		}
		if (arg0.get("--compact") != null) {
			compactGenericCol = true;
		}
		if (arg0.get("--log-generic-col") != null) {
			logGenericCol = true;
		}
		// space delimited list of fields to omit in generic col
		if (arg0.get("--omit-fields") != null) {
		  omittedFields = arg0.get("--omit-fields");
		}
		displayClient.configureDisplayClient(arg0);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see castutils.viewer.ViewerCastComponent#start()
	 */
	@Override
	protected void start() {
		entrySet.start();
		displayClient.connectIceClient(this);
		//displayClient.installEventReceiver(); // no callbacks => receiver not needed
		displayClient.initDisplay();
	}

}
