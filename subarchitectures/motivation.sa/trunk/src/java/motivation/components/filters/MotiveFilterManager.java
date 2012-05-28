/**
 * 
 */
package motivation.components.filters;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;
import java.util.concurrent.TimeUnit;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveView;
import motivation.util.WMMotiveView.MotiveStateTransition;
import Ice.ObjectImpl;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;

/**
 * @author marc
 * 
 */
public class MotiveFilterManager extends ManagedComponent {

	private static final int RECHECK_INTERVAL_DEFAULT = 5000;
	private int recheckInterval = RECHECK_INTERVAL_DEFAULT;
	WMEventQueue receiver;
	private WMMotiveView motives;

	List<MotiveFilter> pipe;

	private final List<WMView.ChangeHandler<Motive>> m_completionHandlers;
	private final List<WMView.ChangeHandler<Motive>> m_activationHandlers;

	public MotiveFilterManager() {
		super();
		pipe = new LinkedList<MotiveFilter>();
		motives = WMMotiveView.create(this);

		receiver = new WMEventQueue();

		m_completionHandlers = new LinkedList<WMView.ChangeHandler<Motive>>();
		m_activationHandlers = new LinkedList<WMView.ChangeHandler<Motive>>();

		// new WorkingMemoryChangeReceiver() {
		// public void workingMemoryChanged(WorkingMemoryChange _wmc) {
		// // avoid self calls
		// if (_wmc.src.equals(getComponentID()))
		// return;
		// log("src of motive change: " + _wmc.src);
		// try {
		// lockComponent();
		// lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDO);
		// Motive motive = getMemoryEntry(_wmc.address, Motive.class);
		// MotivePriority priority = checkMotive(motive, _wmc);
		// // if we have to reprioritize
		// if (priority != motive.priority) {
		// switch (priority) {
		// case UNSURFACE: // if the priority is UNSURFACE then
		// // change status
		// motive.status = MotiveStatus.UNSURFACED;
		// break;
		// default:
		// // only if the previous status was UNSURFACED we
		// // change it here to SURFACED
		// if (motive.status == MotiveStatus.UNSURFACED)
		// motive.status = MotiveStatus.SURFACED;
		// break;
		// }
		// motive.priority = priority;
		// overwriteWorkingMemory(_wmc.address, motive);
		// }
		// } catch (DoesNotExistOnWMException e) {
		// println("filter failed to access motive from WM, maybe has been removed... ignore");
		// } catch (UnknownSubarchitectureException e) {
		// println("UnknownSubarchitectureException: this shouldn't happen.");
		// e.printStackTrace();
		// } catch (CASTException e) {
		// println("CASTException in motive filtering ");
		// e.printStackTrace();
		// } finally {
		// try {
		// unlockEntry(_wmc.address);
		// unlockComponent();
		// } catch (CASTException e) {
		// println("CASTException while unlocking: ");
		// e.printStackTrace();
		// }
		// }
		//
		// }
		// };

	}

	public void processChange(WorkingMemoryChange _wmc) {
		// avoid self calls
		if (_wmc.src.equals(getComponentID()))
			return;

		// if the motive was deleted but we're seeing some residual overwrite
		// messages
		if (_wmc.operation == WorkingMemoryOperation.OVERWRITE
				&& !motives.containsKey(_wmc.address)) {
			log("Receiving left-over overwrites, ignoring");
			return;
		}

		debug("src of motive change: " + _wmc.src);
		try {
			lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDO);
			Motive motive = getMemoryEntry(_wmc.address, Motive.class);
			MotivePriority priority = checkMotive(motive, _wmc);
			// if we have to reprioritize
			if (priority != motive.priority) {
				switch (priority) {
				case UNSURFACE: // if the priority is UNSURFACE then
					// change status
					if (motive.status != MotiveStatus.COMPLETED)
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
		} catch (CASTException e) {
			logException("CASTException in motive filtering ", e);
		} finally {
			try {
				unlockEntry(_wmc.address);
			} catch (CASTException e) {
				logException("CASTException in motive filtering ", e);
			}
		}

	}

	@Override
	protected void configure(Map<String, String> arg0) {
		log("configure filter");
		super.configure(arg0);
		String argStr = arg0.get("--recheck-interval");
		if (argStr != null) {
			recheckInterval = Integer.parseInt(argStr);
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
					logException(
							"trying to register for a class that doesn't exist ",
							e);
				} catch (InstantiationException e) {
					logException(e);
				} catch (IllegalAccessException e) {
					logException(e);
				}
			}
		}

		if (arg0.containsKey("--delete-on-complete")) {
			log("Will delete motives when marked as complete");
			addMotiveCompletionHandler(new WMView.ChangeHandler<Motive>() {

				@Override
				public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
						WorkingMemoryChange wmc, Motive newEntry,
						Motive oldEntry) throws CASTException {

					println("Motive transitioned to completed");
					println("Deleting achieved motive "
							+ newEntry.goal.goalString);

					WorkingMemoryAddress wma = wmc.address;

					try {
						// this seems to be the pattern elsewhere
						lockEntry(wma, WorkingMemoryPermissions.LOCKEDOD);
						// delete from wm
						motives.remove(wma);

					} catch (CASTException e) {
						getLogger().warn(
								"CASTException when deleting achieved motive: "
										+ e.message);
					}
				}

			});
		}

	}

	public void addFilter(MotiveFilter mf) {
		mf.setManager(this);
		pipe.add(mf);
	}

	public void addMotiveCompletionHandler(ChangeHandler<Motive> _handler) {
		m_completionHandlers.add(_handler);
	}

	public void addMotiveActivationHandler(ChangeHandler<Motive> _handler) {
		m_activationHandlers.add(_handler);
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

		// register a handler for completed motives. this should be before
		// starting filters in case they also add filters in their start methods
		// (so this component gets to act first)
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.WILDCARD, MotiveStatus.COMPLETED),
				new WMView.ChangeHandler<Motive>() {

					@Override
					public void entryChanged(
							Map<WorkingMemoryAddress, Motive> map,
							WorkingMemoryChange wmc, Motive newEntry,
							Motive oldEntry) throws CASTException {

						for (WMView.ChangeHandler<Motive> handler : m_completionHandlers) {
							// dispatch to others
							handler.entryChanged(map, wmc, newEntry, oldEntry);
						}
					}

				});

		// register a handler for activated motives. this should be before
		// starting filters in case they also add filters in their start methods
		// (so this component gets to act first)
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.WILDCARD, MotiveStatus.ACTIVE),
				new WMView.ChangeHandler<Motive>() {

					@Override
					public void entryChanged(
							Map<WorkingMemoryAddress, Motive> map,
							WorkingMemoryChange wmc, Motive newEntry,
							Motive oldEntry) throws CASTException {

						for (WMView.ChangeHandler<Motive> handler : m_activationHandlers) {
							// dispatch to others
							handler.entryChanged(map, wmc, newEntry, oldEntry);
						}
					}

				});
		
		
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
			receiver.add(wmc);
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
				WorkingMemoryChange wmc;
				if (recheckInterval > 0)
					wmc = receiver.poll(recheckInterval, TimeUnit.MILLISECONDS);
				else
					wmc = receiver.take();

				if (wmc == null) {
					getLogger().debug(
							"time elapsed without event. check everything");
					checkAll();
				} else {
					getLogger().debug(
							"got an update for motive "
									+ CASTUtils.toString(wmc));
					processChange(wmc);
				}
			} catch (CASTException e) {
				logException(e);
			} catch (InterruptedException e) {
				logException(e);
			}
		}
	}

	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		MotivePriority result = MotivePriority.HIGH;
		for (MotiveFilter filter : pipe) {
			MotivePriority filterResult = filter.checkMotive(motive, wmc);
			assert (filterResult != null);

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
