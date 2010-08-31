package celmarchitecture.subarchitectures.monitors;

import java.util.Date;
import java.util.Vector;
import java.util.Map;
import java.util.HashMap;

import cast.cdl.WorkingMemoryOperation;

import elm.event.EventSpecificFeatures;
import celmarchitecture.global.GlobalSettings;


/**
 * EventMonitorPlugin is an interface defining the prototypes of methods
 * which can deduce event information (sensu ELM) from CAST WM changes.
 * The EventMonitorDefaultPlugin provides a default implementation of the 
 * EventMonitorPlugin interface. It can, however, only extract some rough
 * event type-general information. You are encouraged to provide more 
 * specific information than this in your plugins.
 *
 * @see PBMonitor
 *
 * @author Dennis Stachowicz
 */
public class EventMonitorDefaultPlugin implements EventMonitorPlugin {

	// stuff related to Marc's WM Viewer Plugins
	public static final String wmViewerPluginPackageName     = "motivation.util.viewer.plugins";
	public static final String wmViewerPluginSuffix          = "Info";

	public static final String esfKeyPrefix                  = "key-";
	public static final String esfWMOperationKey             = "WM-OP";

	private boolean localVerbose                             = false;
	private boolean verbose                                  = GlobalSettings.verbose || localVerbose;

	private	Map<Class<?>, motivation.util.viewer.plugins.Plugin> wmViewerDispatcherMap;

	public EventMonitorDefaultPlugin() {
		wmViewerDispatcherMap = new HashMap<Class<?>, motivation.util.viewer.plugins.Plugin>();
	}

	/**
	  *  Return the event type indicated by a WM change involving the given parameter iceObject.
	  *  If you do not redefine this method it will just use the class name of the object. (Better than nothing.)  
	  */
	public String getEventType(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject) {
		return iceObject.getClass().getSimpleName();
	}

	/**
	  *  If you can deduct the start time of the event from the given parameter iceObject
	  *  you should do so. If you cannot return null and it will be estimated via the time
	  *  of the registered change event.
	  */
	public Date getStartTime(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject) {
		return null;
	}

	/**
	  *  If you can deduct the end time of the event from the given parameter iceObject
	  *  you should do so. If you cannot return null and it will be estimated via the time
	  *  of the registered change event.
	  */
	public Date getEndTime(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject) {
		return null;
	}

	/**
	  * (optional) if there is a binary payload (modal information) 
	  *  which should be stored with the event it can be specified by overriding this definition
	  */		
	public byte[] getEventSpecificBinaryData(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject) {
		return null;
	}

	/**
	  * (optional)  For other useful event information in string format, 
 	  *  more precisely, as a set of (name, value) pairs of strings.
	  *  If you do not specify anything there, it will be tried whether there
	  *  is a motivation.util.viewer.plugins.Plugin which could at least give us 
          *  something useful. If this is undesired, redefine as "return null" to leave
	  *  out this kind of additional information.
	  */
	public EventSpecificFeatures getEventSpecificFeatures(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject) {

		EventSpecificFeatures esfs = new EventSpecificFeatures();
				
		esfs.addKeyValuePair(esfWMOperationKey, getWMOperationName(wmOperation));
						
		motivation.util.viewer.plugins.Plugin wmViewerPlugin = findWMViewerPlugin(iceObject.getClass());

		if (wmViewerPlugin != null) {
			Vector<Object> objs = wmViewerPlugin.toVector(iceObject);
			for (int i = 0; i < objs.size(); i++)
				esfs.addKeyValuePair(esfKeyPrefix + (i+1), objs.get(i).toString()); 
		}
					
		return esfs;
	}

	protected String getWMOperationName(WorkingMemoryOperation wmOperation) {

		String wmOpName = "unkwown";		

		if (wmOperation.value() == WorkingMemoryOperation.ADD.value()) 
			wmOpName = "ADD";
		else if (wmOperation.value() == WorkingMemoryOperation.OVERWRITE.value()) 
			wmOpName = "OVERWRITE";
		else if (wmOperation.value() == WorkingMemoryOperation.DELETE.value()) 
			wmOpName = "DELETE";

		return wmOpName;
	} 


	protected static String getWMViewerPluginFullName(String simpleName) {
		return wmViewerPluginPackageName + "." + simpleName + wmViewerPluginSuffix;
	}


	/*
	 *  TEMPORARY SOLUTION:
	 *  This is (pretty much) Marc's code copied. In his class this is private 
	 *  (for understandable reasons). But considering that it is now also used 
         *  elsewhere (here) it would be much better if he could provide a public interface for this
         *  so that the code is well encapsulated and not duplicated (bad if it changes!)
	 *  (At least there could be a public method returning the full path to plugins.)
	 */
	protected motivation.util.viewer.plugins.Plugin findWMViewerPlugin(Class<?> origType) {
		Class<?> oType = origType;
		if (wmViewerDispatcherMap.containsKey(oType))
			return wmViewerDispatcherMap.get(oType);
		// if not yet found, look for supertype plugins
		motivation.util.viewer.plugins.Plugin pluginToCall = null;
		while (pluginToCall == null) {
			String simpleName = oType.getSimpleName();
			String fullName = getWMViewerPluginFullName(simpleName);
			try {
				if (verbose) 
					System.err.println("trying to load class " + fullName);
				ClassLoader.getSystemClassLoader().loadClass(fullName);
				pluginToCall = (motivation.util.viewer.plugins.Plugin) Class.forName(fullName).newInstance();
				if (verbose) 
					System.err.println("succeeded... memorize plugin " + fullName
						   	   + "for type " + oType.getSimpleName());
				break;
			} catch (ClassNotFoundException e) {
				if (verbose) 
					System.err.println("no class " + fullName + " exists.");
			} catch (InstantiationException e) {
				e.printStackTrace();
				if (GlobalSettings.exitOnException)
					System.exit(GlobalSettings.exitValueOnException);
			} catch (IllegalAccessException e) {
				e.printStackTrace();
				if (GlobalSettings.exitOnException)
					System.exit(GlobalSettings.exitValueOnException);
			}
			oType = oType.getSuperclass();
			if (oType == null) // if no superclass exists, we have to give up
				break;
			if (oType == Ice.Object.class) // we don't need to look up
				// further
				break;

		}
		wmViewerDispatcherMap.put(origType, pluginToCall);
		return pluginToCall;

	}

}



