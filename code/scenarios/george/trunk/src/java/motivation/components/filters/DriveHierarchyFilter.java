package motivation.components.filters;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeQuestionMotive;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;

/**
 * 
 * A filter that allows only motives of the highest visible priority through.
 * 
 * @author nah
 * 
 */
public class DriveHierarchyFilter implements MotiveFilter {

	private static final int UNKNOWN_CLASS_VALUE = Integer.MAX_VALUE;

	private MotiveFilterManager m_component;

	private final ArrayList<Set<Class<? extends Motive>>> m_driveHierarchy;

	int m_activeLevel = Integer.MAX_VALUE;

	public DriveHierarchyFilter() {
		m_driveHierarchy = new ArrayList<Set<Class<? extends Motive>>>();
	}

	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {

		Class<? extends Motive> motiveCls = motive.getClass();
		int priority = getPriority(motiveCls);
		if (priority == UNKNOWN_CLASS_VALUE) {
			m_component.getLogger().warn("Unknown motive class: " + motiveCls,
					m_component.getLogAdditions());
			return null;
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
	}

	private void addPrioritySet(Class<? extends Motive>... _motiveClasses) {
		HashSet<Class<? extends Motive>> motiveClasses = new HashSet<Class<? extends Motive>>(
				_motiveClasses.length);

		for (Class<? extends Motive> cls : _motiveClasses) {
			motiveClasses.add(cls);
		}

		m_driveHierarchy.add(motiveClasses);
	}

	private int getPriority(Class<? extends Motive> _motiveCls) {
		int priority = UNKNOWN_CLASS_VALUE;
		for (int i = 0; i < m_driveHierarchy.size(); i++) {
			if (m_driveHierarchy.get(i).contains(_motiveCls)) {
				priority = i;
				break;
			}
		}
		return priority;
	}

	@SuppressWarnings("unchecked")
	@Override
	public void configure(Map<String, String> _config) {
		// define priority levels, highest first
		addPrioritySet(TutorInitiativeLearningMotive.class,
				TutorInitiativeQuestionMotive.class);
		addPrioritySet(AnalyzeProtoObjectMotive.class,
				LearnObjectFeatureMotive.class);
	}

}
