/**
 * 
 */
package motivation.components.filters;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.StringTokenizer;

import Ice.ObjectImpl;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.TestMotive;
import motivation.util.WMMotiveSet;
import motivation.util.WMMotiveSet.MotiveStateTransition;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
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

	WorkingMemoryChangeReceiver receiver;
	WMMotiveSet motives;

	List<MotiveFilter> pipe;

	public MotiveFilterManager() {
		super();
		pipe = new LinkedList<MotiveFilter>();
		motives = WMMotiveSet.create(this);
	}

	@Override
	protected void configure(Map<String, String> arg0) {
		// TODO Auto-generated method stub
		log("configure filter");
		super.configure(arg0);
		String subscrStr = arg0.get("--filter");
		if (subscrStr != null) {
			StringTokenizer st = new StringTokenizer(subscrStr, ",");
			while (st.hasMoreTokens()) {
				String className = st.nextToken();
				className = this.getClass().getPackage().getName() + "."
						+ className.trim();
				try {
					log("add type '" + className + "'");
					ClassLoader.getSystemClassLoader().loadClass(className);
					addFilter((MotiveFilter) Class.forName(className)
							.newInstance());
				} catch (ClassNotFoundException e) {
					println("trying to register for a class that doesn't exist.");
					e.printStackTrace();
				} catch (InstantiationException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					// TODO Auto-generated catch block
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
		// TODO Auto-generated method stub
		super.start();

		motives.start();

		receiver = new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				// avoid self calls
				if (_wmc.src.equals(getComponentID()))
					return;
				log("src of motive change: " + _wmc.src);
				try {
					lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDO);
					Motive motive = getMemoryEntry(_wmc.address, Motive.class);
					switch (motive.status) {
					case UNSURFACED:
						log("check unsurfaced motive " + motive.toString());
						MotivePriority priority = shouldBeSurfaced(motive, _wmc);
						if (priority != MotivePriority.UNSURFACE) {
							log("-> surfaced motive " + motive.toString());
							motive.status = MotiveStatus.SURFACED;
							motive.priority = priority;
							overwriteWorkingMemory(_wmc.address, motive);
						}
						break;
					default:
						log("check surfaced motive " + motive.toString());
						if (shouldBeUnsurfaced(motive, _wmc)) {
							log("-> unsurfaced motive " + motive.toString());
							motive.status = MotiveStatus.UNSURFACED;
							motive.priority = MotivePriority.UNSURFACE;
							overwriteWorkingMemory(_wmc.address, motive);
						}
						break;
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

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Motive.class,
				WorkingMemoryOperation.ADD), receiver);
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Motive.class,
				WorkingMemoryOperation.OVERWRITE), receiver);

	}

	public void checkAll() throws CASTException {

		for (ObjectImpl obj : motives.values()) {
			Motive motive = (Motive) obj;
			WorkingMemoryChange wmc = new WorkingMemoryChange();
			wmc.address = motive.thisEntry;
			wmc.operation = WorkingMemoryOperation.OVERWRITE;
			wmc.src = "explicit self-trigger";
			receiver.workingMemoryChanged(wmc);
		}

	}

	public MotivePriority shouldBeSurfaced(Motive motive, WorkingMemoryChange wmc) {
		MotivePriority result = MotivePriority.HIGH;
		for (MotiveFilter filter : pipe) {
			MotivePriority filterResult = filter.shouldBeSurfaced(motive,wmc);
			// if one filter rejects, reject this motive
			if (filterResult == MotivePriority.UNSURFACE)
				return MotivePriority.UNSURFACE;
			// otherwise always provide the lowest priority
			if (filterResult.value() < result.value())
				result = filterResult;
		}
		return result;
	}

	public boolean shouldBeUnsurfaced(Motive motive, WorkingMemoryChange wmc) {
		boolean result = false;
		for (MotiveFilter filter : pipe) {
			result = result || filter.shouldBeUnsurfaced(motive,wmc);
		}
		return result;
	}

}
