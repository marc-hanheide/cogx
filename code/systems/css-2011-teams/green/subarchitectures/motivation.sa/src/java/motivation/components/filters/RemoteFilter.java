/**
 * 
 */
package motivation.components.filters;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.RemoteFilterServer;
import motivation.slice._RemoteFilterServerOperations;
import motivation.slice._RemoteFilterServerTie;
import Ice.Current;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class RemoteFilter implements MotiveFilter,
		_RemoteFilterServerOperations {

	MotiveFilterManager filterManager = null;
	Map<Class<?>, MotivePriority> priorities;
	//RemoteFilterServer server;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.filters.MotiveFilter#checkMotive(motivation.slice
	 * .Motive, cast.cdl.WorkingMemoryChange)
	 */
	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		MotivePriority result = priorities.get(motive.getClass());
		if (result == null) {
			result = MotivePriority.UNSURFACE;
		}
		return result;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.filters.MotiveFilter#setManager(motivation.components
	 * .filters.MotiveFilterManager)
	 */
	@Override
	public void setManager(MotiveFilterManager motiveFilterManager) {
		filterManager = motiveFilterManager;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see motivation.components.filters.MotiveFilter#start()
	 */
	@Override
	public void start() {
		filterManager.log("registering new IceServer "
				+ RemoteFilterServer.class.getCanonicalName());
		filterManager.registerIceServer(RemoteFilterServer.class,
				new _RemoteFilterServerTie(this));
	}

	@Override
	public void setPriority(String motiveType, String priority, Current current) {
		filterManager.log("Ice interface method setPriority called");
		MotivePriority newMP = null;
		for (MotivePriority mp : MotivePriority.values()) {
			if (mp.toString().equalsIgnoreCase(priority)) {
				newMP = mp;
			}
		}
		if (newMP == null) {
			filterManager.getLogger().error(
					"unknown priority asked for: " + priority);
			throw new IllegalArgumentException("unknown priority asked for: "
					+ priority);
		}
		try {
			// assume all Motive sub-classes are in the same package
			Class<?> motiveClass = Class.forName(Motive.class.getPackage()
					.getName()+"."
					+ motiveType);
			filterManager.log("set priority for class "
					+ motiveClass.getSimpleName() + " to " + newMP.name());
			priorities.put(motiveClass, newMP);
		} catch (ClassNotFoundException e) {
			filterManager.getLogger().error(
					"could not find class for motive type " + motiveType, e);
			throw new IllegalArgumentException("unknown type asked for: "
					+ motiveType);
		}
		try {
			filterManager.checkAll();
		} catch (CASTException e) {
			filterManager.getLogger().error(
					"exception while checking all motives after change " + motiveType, e);
			
		}

	}

	/**
	 * @param priorities
	 */
	public RemoteFilter() {
		super();
		this.priorities = Collections
				.synchronizedMap(new HashMap<Class<?>, MotivePriority>());
	}

	@Override
	public void configure(Map<String, String> arg0) {
		// TODO Auto-generated method stub
		
	}

}
