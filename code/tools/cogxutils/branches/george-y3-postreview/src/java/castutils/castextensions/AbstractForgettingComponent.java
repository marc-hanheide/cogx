package castutils.castextensions;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.StringTokenizer;
import java.util.Map.Entry;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.CASTTimeUtil;

public abstract class AbstractForgettingComponent<TYPE extends Ice.Object> extends
		ManagedComponent implements WorkingMemoryChangeReceiver {

	private static final int CHECK_FREQUENCY_MS = 1000;
	private static final int DEFAULT_MAX_AGE = 3000;
	protected final Set<Class<? extends TYPE>> registeredTypes = new HashSet<Class<? extends TYPE>>();
	final Map<WorkingMemoryAddress, CASTTime> lastUpdateMap = Collections
			.synchronizedMap(new HashMap<WorkingMemoryAddress, CASTTime>());
	private long maxAge;

	@Override
	public void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		switch (wmc.operation) {
		case ADD:
		case OVERWRITE:
			if (shouldBeHandled(wmc)) {
				lastUpdateMap.put(wmc.address, this.getCASTTime());
			}
			break;
		case DELETE:
			lastUpdateMap.remove(wmc.address);
			break;
		}

	}

	abstract protected boolean shouldBeHandled(WorkingMemoryChange wmc) throws CASTException;
	
	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected void configure(Map<String, String> config) {
		String ageStr = config.get("--max-age");
		if (ageStr != null) {
			this.maxAge = Long.valueOf(ageStr);
		} else {
			this.maxAge = DEFAULT_MAX_AGE;
		}

		String subscrStr = config.get("--subscribe");
		if (subscrStr != null) {
			StringTokenizer st = new StringTokenizer(subscrStr, ",");
			while (st.hasMoreTokens()) {
				String className = st.nextToken();
				className = className.trim();
				try {
					System.out.println("add type '" + className + "'");
					ClassLoader.getSystemClassLoader().loadClass(className);
					registeredTypes.add((Class<? extends TYPE>) Class
							.forName(className));
				} catch (ClassNotFoundException e) {
					println("trying to register for a class that doesn't exist.");
					e.printStackTrace();
				}
			}
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {

		while (isRunning()) {
			try {
				CASTTime now = getCASTTime();
				Set<Entry<WorkingMemoryAddress, CASTTime>> tempSet = new HashSet<Entry<WorkingMemoryAddress, CASTTime>>(
						lastUpdateMap.entrySet());
				for (Entry<WorkingMemoryAddress, CASTTime> e : tempSet) {
					long diff = CASTTimeUtil.diff(now, e.getValue());
					if (diff > maxAge) {
						log("delete " + CASTUtils.toString(e.getKey())
								+ " because it was not updated for more than "
								+ maxAge + " ms.");
						deleteFromWorkingMemory(e.getKey());
					}
				}
				this.sleepComponent(CHECK_FREQUENCY_MS);
			} catch (CASTException e) {
				logException(e);
			}
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		for (Class<? extends Ice.Object> t : registeredTypes) {
			addChangeFilter(ChangeFilterFactory.createTypeFilter(t,
					WorkingMemoryOperation.WILDCARD), this);
		}
	}

}
