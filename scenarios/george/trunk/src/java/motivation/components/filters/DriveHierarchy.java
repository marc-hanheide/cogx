package motivation.components.filters;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import motivation.slice.Motive;

public class DriveHierarchy {
	
	private final ArrayList<Set<Class<? extends Motive>>> m_driveHierarchy;

	public static final int UNKNOWN_CLASS_VALUE = Integer.MAX_VALUE;

	public DriveHierarchy() {
		m_driveHierarchy = new ArrayList<Set<Class<? extends Motive>>>();
	}
	
	public void addPrioritySet(Class<? extends Motive>... _motiveClasses) {
		HashSet<Class<? extends Motive>> motiveClasses = new HashSet<Class<? extends Motive>>(
				_motiveClasses.length);

		for (Class<? extends Motive> cls : _motiveClasses) {
			motiveClasses.add(cls);
		}

		m_driveHierarchy.add(motiveClasses);
	}

	public int getPriority(Class<? extends Motive> _motiveCls) {
		int priority = UNKNOWN_CLASS_VALUE;
		for (int i = 0; i < m_driveHierarchy.size(); i++) {
			if (m_driveHierarchy.get(i).contains(_motiveCls)) {
				priority = i;
				break;
			}
		}
		return priority;
	}

}


