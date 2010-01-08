/**
 * 
 */
package motivation.components.filters;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice._RemoteFilterServerOperations;
import Ice.Current;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class RemoteFilter implements MotiveFilter,
		_RemoteFilterServerOperations {

	MotiveFilterManager component = null;
	Map<Class<?>, MotivePriority> priorities;

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
		return null;
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
		component = motiveFilterManager;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see motivation.components.filters.MotiveFilter#start()
	 */
	@Override
	public void start() {
		// TODO have to implement code here that uses CASTComponent.registerIceServer

	}

	@Override
	public void setPriority(String motiveType, String priority, Current current) {
		component.log("Ice interface method setPriority called");
		MotivePriority newMP = null;
		for (MotivePriority mp : MotivePriority.values()) {
			if (mp.name().equalsIgnoreCase(priority)) {
				mp = newMP;
			}
		}
		if (newMP == null) {
			component.getLogger().error("unknown priority asked for: "
				+ priority);
			throw new IllegalArgumentException("unknown priority asked for: "
					+ priority);
		}
		try {
			// TODO: quite a hack to set the package name constant here
			Class<?> motiveClass = Class.forName("motivation.slice."
					+ motiveType);
			component.log("set priority for class " + motiveClass.getSimpleName() + " to " + newMP.name());
			priorities.put(motiveClass, newMP);
		} catch (ClassNotFoundException e) {
			component.getLogger().error("could not find class for motive type" + motiveType,e);
			throw new IllegalArgumentException("unknown type asked for: "
					+ motiveType);
		}

	}

	/**
	 * @param priorities
	 */
	public RemoteFilter() {
		super();
		this.priorities = Collections.synchronizedMap(new HashMap<Class<?>, MotivePriority> ());
	}

}
