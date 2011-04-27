/**
 * 
 */
package motivation.components.filters;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveView;
import Ice.ObjectImpl;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;

/**
 * @author marc
 * 
 */
public class MotiveFilterManager extends ManagedComponent {

	private static final int RECHECK_INTERVAL_DEFAULT = 5000;
	private int recheckInterval = RECHECK_INTERVAL_DEFAULT;
	WorkingMemoryChangeReceiver receiver;
	WMMotiveView motives;

	List<MotiveFilter> pipe;

	public MotiveFilterManager() {
		super();
		pipe = new LinkedList<MotiveFilter>();
		motives = WMMotiveView.create(this);

		receiver = new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				// avoid self calls
				if (_wmc.src.equals(getComponentID()))
					return;
				log("src of motive change: " + _wmc.src);
				try {
					lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDO);
					Motive motive = getMemoryEntry(_wmc.address, Motive.class);
					MotivePriority priority = checkMotive(motive, _wmc);
					// if we have to reprioritize
					if (priority != motive.priority) {
						switch (priority) {
						case UNSURFACE: // if the priority is UNSURFACE then
							// change status
							motive.status = MotiveStatus.UNSURFACED;
							break;
						default:
							// only if the previous status was UNSURFACED we
							// change it here to SURFACED
							if (motive.status == MotiveStatus.UNSURFACED)
								motive.status = MotiveStatus.SURFACED;
							break;
						}
						motive.priority = priority;
						overwriteWorkingMemory(_wmc.address, motive);
					}
				} catch (DoesNotExistOnWMException e) {
					println("filter failed to access motive from WM, maybe has been removed... ignore");
				} catch (UnknownSubarchitectureException e) {
					println("UnknownSubarchitectureException: this shouldn't happen.");
					e.printStackTrace();
				} catch (CASTException e) {
					println("CASTException in motive filtering ");
					e.printStackTrace();
				} finally {
					try {
						unlockEntry(_wmc.address);
					} catch (CASTException e) {
						println("CASTException while unlocking: ");
						e.printStackTrace();
					}
				}

			}
		};

	}

	@Override
	protected void configure(Map<String, String> arg0) {
		log("configure filter");
		super.configure(arg0);
		String argStr = arg0.get("--recheck-interval");
		if (argStr != null) {
			recheckInterval=Integer.parseInt(argStr);
		}
		argStr = arg0.get("--filter");
		if (argStr != null) {
			StringTokenizer st = new StringTokenizer(argStr, ",");
			while (st.hasMoreTokens()) {
				String className = st.nextToken();
				className = this.getClass().getPackage().getName() + "."
				+ className.trim();
				try {
					log("add type '" + className + "'");
					ClassLoader.getSystemClassLoader().loadClass(className);
					MotiveFilter filter = (MotiveFilter) Class.forName(
							className).newInstance();
					filter.configure(arg0);
					addFilter(filter);
				} catch (ClassNotFoundException e) {
					println("trying to register for a class that doesn't exist.");
					e.printStackTrace();
				} catch (InstantiationException e) {
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					e.printStackTrace();
				}
			}
		}
	}

	public void addFilter(MotiveFilter mf) {
		mf.setManager(this);
		pipe.add(mf);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {

		try {
			motives.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}

		for (MotiveFilter f : pipe) {
			f.start();
		}

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Motive.class,
				WorkingMemoryOperation.ADD), receiver);
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Motive.class,
				WorkingMemoryOperation.OVERWRITE), receiver);

		super.start();

	}

	public void checkAll() throws CASTException {

		for (ObjectImpl obj : new HashSet<ObjectImpl>(motives.values())) {
			Motive motive = (Motive) obj;
			WorkingMemoryChange wmc = new WorkingMemoryChange();
			wmc.address = motive.thisEntry;
			wmc.operation = WorkingMemoryOperation.OVERWRITE;
			wmc.src = "explicit self-trigger";
			receiver.workingMemoryChanged(wmc);
		}

	}

	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		if (recheckInterval>0) {
			while(isRunning()) {
				sleepComponent(recheckInterval);
				try {
					checkAll();
				} catch (CASTException e) {
					logException(e);
				}
			}
		}
	}

	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		MotivePriority result = MotivePriority.HIGH;
		for (MotiveFilter filter : pipe) {
			MotivePriority filterResult = filter.checkMotive(motive, wmc);
			// if one filter rejects, reject this motive
			if (filterResult == MotivePriority.UNSURFACE)
				return MotivePriority.UNSURFACE;
			// otherwise always provide the lowest priority
			if (filterResult.ordinal() < result.ordinal())
				result = filterResult;
		}
		return result;
	}
}
