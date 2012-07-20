package motivation.components.filters;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView.ChangeHandler;

/**
 * 
 * A filter that allows only motives of the highest visible priority through.
 * 
 * @author nah
 * 
 */
public class DriveHierarchyFilter implements MotiveFilter,
		ChangeHandler<Motive> {

	int m_activeLevel = Integer.MAX_VALUE;

	private MotiveFilterManager m_component;

	private DelayFilterDisplayClient m_display;

	private final DriveHierarchy m_driveHierarchy = GeorgeDriveConfig
			.getGeorgeDriveHierarchy();

	public DriveHierarchyFilter() {

	}

	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {

		Class<? extends Motive> motiveCls = motive.getClass();

		int priority = m_driveHierarchy.getPriority(motiveCls);
		if (priority == DriveHierarchy.UNKNOWN_CLASS_VALUE) {
			m_component.getLogger().warn("Unknown motive class: " + motiveCls,
					m_component.getLogAdditions());
			return null;
		}

		// ignore completed etc. motives as we don't want them to influence
		// drive calculations
		if (ignoreThis(motive)) {
			return motive.priority;
		}

		// if in a surfaced class, return surface
		if (priority == m_activeLevel) {
			return MotivePriority.NORMAL;
		}
		// if in an unsurfaced class, return unsurface
		else if (priority > m_activeLevel) {
			return MotivePriority.UNSURFACE;
		}
		// if in a class higher than surfaced update priority and recheck all
		else {
			// set new level to the level for this motive
			m_activeLevel = priority;
			m_component.println("Switched to priority level " + m_activeLevel
					+ " for motive class " + motiveCls);
			m_display.updateActiveLevel(m_activeLevel);

			// schedule a recheck, which will come back to this motive
			try {
				m_component.checkAll();
			} catch (CASTException e) {
				m_component.logException(e);
			}
			return motive.priority;
		}

	}

	@Override
	public void configure(Map<String, String> _config) {
		m_display = DelayFilterDisplayClient.getClient(_config);
		m_display.setDriveHierarchy(m_driveHierarchy);
	}

	/**
	 * Called when a motive has completed.
	 */
	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
			WorkingMemoryChange wmc, Motive newEntry, Motive oldEntry)
			throws CASTException {

		// The motives remaining in map are what is left on WM. The new active
		// level should be the maximum represented one there, else reset to
		// unknown.

		// m_component.println("checking on deactivation");

		// int newLevel = DriveHierarchy.UNKNOWN_CLASS_VALUE;
		// for (Motive mtv : map.values()) {
		// // ignore motives that are done or impossible, and the one that just
		// changed
		// if (!ignoreThis(mtv) && !newEntry.thisEntry.equals(mtv.thisEntry)) {
		//
		// m_component.println("checking on deactivation: " + mtv.getClass());
		//
		// newLevel = Math.min(
		// m_driveHierarchy.getPriority(mtv.getClass()), newLevel);
		// }
		// }

		// if (newLevel != m_activeLevel) {
		// m_activeLevel = newLevel;
		// if (m_activeLevel != DriveHierarchy.UNKNOWN_CLASS_VALUE) {
		// m_component
		// .println("after completion, switching active level to "
		// + m_activeLevel);
		// } else {
		m_component
				.println("after completion, switching active level to allow all");

		// }

		m_activeLevel = DriveHierarchy.UNKNOWN_CLASS_VALUE;
		m_display.updateActiveLevel(m_activeLevel);

		try {
			// trigger recheck after level change
			m_component.checkAll();
		} catch (CASTException e) {
			m_component.logException(e);
		}
		// }

	}

	/**
	 * Returns true if this motive should be ignored when setting drive up.
	 * 
	 * @param motive
	 * @return
	 */
	private boolean ignoreThis(Motive motive) {
		if (motive.status == MotiveStatus.COMPLETED) {
			return true;
		} else if (motive.status == MotiveStatus.IMPOSSIBLE) {
			return true;
		} else {
			return false;
		}
	}

	@Override
	public void setManager(MotiveFilterManager motiveFilterManager) {
		m_component = motiveFilterManager;

	}

	@Override
	public void start() {
		m_component.addMotiveCompletionHandler(this);
		m_display.connect(m_component);
		m_display.updateActiveLevel(m_activeLevel);

//		new Thread(new Runnable() {
//
//			@Override
//			public void run() {
//
//				try {
//					Thread.sleep(5000);
//				} catch (InterruptedException e) {
//					e.printStackTrace();
//				}
//				while (m_component.isRunning()) {
//					JOptionPane.showMessageDialog(null, "Reset filter");
//					m_activeLevel = DriveHierarchy.UNKNOWN_CLASS_VALUE;
//					try {
//						m_component.lockComponent();
//						m_component.checkAll();
//					} catch (Throwable t) {
//						m_component.logException("on reset", t);
//					} finally {
//						m_component.unlockComponent();
//					}
//				}
//
//			}
//		}).start();
	}

}
