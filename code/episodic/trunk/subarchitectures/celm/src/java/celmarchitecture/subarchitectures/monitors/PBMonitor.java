package celmarchitecture.subarchitectures.monitors;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

import java.util.Date;
import java.util.Vector;
import java.util.Map;
import java.util.HashMap;
import java.util.StringTokenizer;
import java.text.SimpleDateFormat;

import Ice.ObjectImpl;

import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.CASTTimeConverter;
import celmarchitecture.global.EventTypeNames;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.subarchitectures.abstr.SimpleAbstractWMMonitor;


/**
 * PBMonitor is ...
 * 
 * @author Dennis Stachowicz
 */
public class PBMonitor extends SimpleAbstractWMMonitor {

	public static final String subDirName               = "plugins";
	public static final String pluginSuffix             = "MonitorPlugin";

	private boolean localVerbose                        = true; // false;
	private boolean verbose                             = GlobalSettings.verbose || localVerbose;

	private EventMonitorDefaultPlugin defaultPlugin     = new EventMonitorDefaultPlugin();
	private Map<Class<?>, EventMonitorPlugin> objectDispatcherMap;
	
	private Vector<Class<? extends ObjectImpl>> classesToRegister = new Vector<Class<? extends ObjectImpl>>();

	public PBMonitor() {
		super();
		objectDispatcherMap = new HashMap<Class<?>, EventMonitorPlugin>();
		// objectDispatcherMap.put( , );
	}

	
        @SuppressWarnings("unchecked")
        @Override
        protected void configure(Map<String, String> config) {
             
                super.configure(config);
                String subscrStr = config.get("--subscribe");
                
                if (subscrStr != null) {
                        StringTokenizer st = new StringTokenizer(subscrStr, ",");
                        while (st.hasMoreTokens()) {
                                String className = st.nextToken();
                                className=className.trim();
                                try {
                                        System.out.println("add type '" + className+"'");
                                        ClassLoader.getSystemClassLoader().loadClass(className);
                                        classesToRegister.add((Class<? extends ObjectImpl>) Class.forName(className));
                                } catch (ClassNotFoundException e) {
                                        println("trying to register for a class that doesn't exist.");
                                        e.printStackTrace();
                                        if (GlobalSettings.exitOnException)
						System.exit(GlobalSettings.exitValueOnException);
                                }
                        }
                }
        }
	

	@Override
	public void start() {
		super.start();

		// try {
			WorkingMemoryChangeReceiver wmcr = new WorkingMemoryChangeReceiver() {

				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					// log(CASTUtils.toString(_wmc));
					processWMChange(_wmc);
				}
			};

		
			for (Class<? extends ObjectImpl> c : classesToRegister) {
				// addGlobalAddOverwriteFilter(c, wmcr);
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
						WorkingMemoryOperation.ADD), wmcr);
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
						WorkingMemoryOperation.OVERWRITE), wmcr);
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
						WorkingMemoryOperation.DELETE), wmcr);		
			}		

		/*
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
		*/

	}

	protected void processWMChange(WorkingMemoryChange wmc) {

		try {
			CASTData<?> wme = getWorkingMemoryEntry(wmc.address);
	
			ObjectImpl data = (ObjectImpl) wme.getData();
	
			EventMonitorPlugin plugin = findPlugin(data.getClass());

			String eventType = plugin.getEventType(wmc.operation, data);

			if (eventType == null || eventType.equals(""))
				return;  // this is a "non-event" 
			
			addPartialEvent(eventType,
					plugin.getEventSpecificBinaryData(wmc.operation, data), 
					plugin.getStartTime(wmc.operation, data),
					plugin.getEndTime(wmc.operation, data),
					plugin.getEventSpecificFeatures(wmc.operation, data));  		

		} catch (SubarchitectureComponentException e) {
			if (verbose)
				e.printStackTrace();
			// 
			// getWorkingMemoryEntry() above might fail anytime
			// but this is not serious. So we do not exit here.
			// 
			// if (GlobalSettings.exitOnException)
			// 	System.exit(GlobalSettings.exitValueOnException);
		}
	
	}


	private String getPluginFullName(String simpleName) {
		return this.getClass().getPackage().getName()
			+ "." + subDirName + "." + simpleName + pluginSuffix;
	}

	/**
	 *  Plugin lookup code strongly "inspired" ;-) by Marc's WM viewer.
	 */
	private EventMonitorPlugin findPlugin(Class<?> origType) {
		Class<?> oType = origType;
		if (objectDispatcherMap.containsKey(oType))
			return objectDispatcherMap.get(oType);
		// if not yet found, look for supertype plugins
		EventMonitorPlugin pluginToCall = null;
		while (pluginToCall == null) {
			String simpleName = oType.getSimpleName();
			String fullName = getPluginFullName(simpleName);
			try {
				if (verbose) 
					println("trying to load class " + fullName);
				ClassLoader.getSystemClassLoader().loadClass(fullName);
				pluginToCall = (EventMonitorPlugin) Class.forName(fullName).newInstance();
				if (verbose) 
					println("succeeded... memorize plugin " + fullName
						+ "for type " + oType.getSimpleName());
				break;
			} catch (ClassNotFoundException e) {
				if (verbose) 
					println("no class " + fullName + " exists.");
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
			if (oType == null || oType == Ice.Object.class) {
				// further
				pluginToCall = defaultPlugin;
				break;
			}

		}
		objectDispatcherMap.put(origType, pluginToCall);
		return pluginToCall;

	}
	

}
