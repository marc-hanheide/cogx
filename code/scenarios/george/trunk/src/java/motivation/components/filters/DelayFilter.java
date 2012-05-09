package motivation.components.filters;

import java.util.HashMap;
import java.util.Map;

import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import cast.cdl.WorkingMemoryChange;

public class DelayFilter implements MotiveFilter {

	private final HashMap<Class<? extends Motive>, Integer> m_delayMap;
	private final HashMap<Class<? extends Motive>, Long> m_activeDelays;
	private final HashMap<Class<? extends Motive>, Long> m_postDelays;
	private MotiveFilterManager m_component;

	public DelayFilter() {
		m_delayMap = new HashMap<Class<? extends Motive>, Integer>();
		m_activeDelays = new HashMap<Class<? extends Motive>, Long>();
		m_postDelays = new HashMap<Class<? extends Motive>, Long>();
	}

	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		Integer delay = m_delayMap.get(motive.getClass());
		MotivePriority priority = MotivePriority.UNSURFACE;

		// is there a delay at all?
		if (delay == null) {
			priority = MotivePriority.NORMAL;
			m_component.log("no delay for " + motive.getClass());
		} else {

			// if there is, check whether we're past it
			Long postDelay = m_postDelays.get(motive.getClass());
			if (postDelay != null) {
				if (System.currentTimeMillis() < postDelay) {
					priority = MotivePriority.NORMAL;
				} else {
					m_postDelays.remove(motive.getClass());
				}
			}

			Long target = m_activeDelays.get(motive.getClass());
			if (target == null) {
				// if no post delay effect was generated
				if (priority == MotivePriority.UNSURFACE) {
					m_component.log("delaying surfacing of "
							+ motive.getClass() + " for " + delay + "ms");
					m_activeDelays.put(motive.getClass(),
							System.currentTimeMillis() + delay);
				}
			} else if (target < System.currentTimeMillis()) {
				m_component.log("surfacing " + motive.getClass()
						+ " after delay of " + delay + "ms");
				priority = MotivePriority.NORMAL;
				m_activeDelays.remove(motive.getClass());
				m_postDelays.put(motive.getClass(), System.currentTimeMillis()
						+ delay);

			}
		}
		return priority;
	}

	@Override
	public void setManager(MotiveFilterManager motiveFilterManager) {
		m_component = motiveFilterManager;
	}

	@Override
	public void start() {
	}

	@Override
	public void configure(Map<String, String> arg0) {
		m_delayMap.put(LearnObjectFeatureMotive.class, 20000);
	}

}
