package motivation.components.filters;

import java.util.Map;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeQuestionMotive;
import si.unilj.fri.cogx.v11n.core.DisplayClient;
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

	private MotiveFilterManager m_component;

	private final DriveHierarchy m_driveHierarchy;

	int m_activeLevel = Integer.MAX_VALUE;

	public DriveHierarchyFilter() {
		m_driveHierarchy = new DriveHierarchy();
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
			return MotivePriority.UNSURFACE;
		}

	}

	@Override
	public void setManager(MotiveFilterManager motiveFilterManager) {
		m_component = motiveFilterManager;

	}

	@Override
	public void start() {
		m_component.addMotiveCompletionHandler(this);
		m_display.connectIceClient(m_component);
		m_display.updateActiveLevel(m_activeLevel);
	}

	@SuppressWarnings("unchecked")
	@Override
	public void configure(Map<String, String> _config) {
		// define priority levels, highest first

		m_driveHierarchy.addPrioritySet(TutorInitiativeLearningMotive.class,
				TutorInitiativeQuestionMotive.class);

		m_driveHierarchy.addPrioritySet(AnalyzeProtoObjectMotive.class,
				LearnObjectFeatureMotive.class);

		m_display.configureDisplayClient(_config);
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

		int newLevel = DriveHierarchy.UNKNOWN_CLASS_VALUE;
		for (Motive mtv : map.values()) {
			// ignore motives that are done or impossible
			if (!ignoreThis(mtv)) {
				newLevel = Math.min(
						m_driveHierarchy.getPriority(mtv.getClass()), newLevel);
			}
		}

		if (newLevel != m_activeLevel) {
			m_activeLevel = newLevel;
			if (m_activeLevel != DriveHierarchy.UNKNOWN_CLASS_VALUE) {
				m_component
						.println("after completion, switching active level to "
								+ m_activeLevel);
			} else {
				m_component
						.println("after completion, switching active level to allow all");

			}

			m_display.updateActiveLevel(m_activeLevel);

			// trigger recheck after level change
			m_component.checkAll();

			try {
				m_component.checkAll();
			} catch (CASTException e) {
				m_component.logException(e);
			}
		}

	}

	private final DriveHierarchyDisplayClient m_display = new DriveHierarchyDisplayClient();

	private class DriveHierarchyDisplayClient extends DisplayClient {
		public void updateActiveLevel(int _level) {
			if (_level == DriveHierarchy.UNKNOWN_CLASS_VALUE) {
				m_display.setHtml("drive.filter", "001",
						"Waiting for any input ");
			} else {
				m_display.setHtml("drive.filter", "001", "Active level is "
						+ _level);
			}
		}
	}
}
