package motivation.components.filters;

import java.util.HashMap;
import java.util.Map;

import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.RobotNonSituatedMotive;
import cast.cdl.WorkingMemoryChange;

/**
 * Delays motives based on their class. This means that once one motive of a
 * particular class has been surfaced after a delay, all others of the same
 * class will be surfaced together
 * 
 * @author nah
 * 
 */
public class DelayFilter implements MotiveFilter {

	// delay prior to activation of given class
	protected final HashMap<Class<? extends Motive>, Integer> m_delayMap;

	// window of activation after a prior activation of the given class
	protected final HashMap<Class<? extends Motive>, Integer> m_postDelayWindowMap;

	protected final HashMap<Class<? extends Motive>, Long> m_activeDelays;
	protected final HashMap<Class<? extends Motive>, Long> m_postDelays;
	protected MotiveFilterManager m_component;

	public DelayFilter() {
		m_delayMap = new HashMap<Class<? extends Motive>, Integer>();
		m_activeDelays = new HashMap<Class<? extends Motive>, Long>();
		m_postDelays = new HashMap<Class<? extends Motive>, Long>();
		m_postDelayWindowMap = new HashMap<Class<? extends Motive>, Integer>();
	}

	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {

		Integer delay = m_delayMap.get(motive.getClass());
		MotivePriority priority = MotivePriority.UNSURFACE;

		long currentSystemTime = System.currentTimeMillis();
		
		// is there a delay configured for this class of motive at all?
		if (delay == null) {
			priority = MotivePriority.NORMAL;
			m_component.log("no delay for " + motive.getClass());
		} else {

			// there is a delay for this class

			// if a motive of this class was previously surfaced then all are
			// allowed to surface for the same amount of time as the delay after
			// the fact

			Long postDelay = m_postDelays.get(motive.getClass());
			// if there is a post-delay window active for this class
			if (postDelay != null) {
				// if we're still in the window, activate the motive
				if (currentSystemTime < postDelay) {
					priority = MotivePriority.NORMAL;
				} else {
					// if we're out of the window, do nothing
					m_postDelays.remove(motive.getClass());
				}
			}

			// Now see if there is currently a delay for this specific motive
			Long target = m_activeDelays.get(motive.getClass());

			
			
			// If this motive has not yet been delayed
			if (target == null) {

				// if no post delay effect was generated
				if (priority == MotivePriority.UNSURFACE) {
					m_component.log("delaying surfacing of "
							+ motive.getClass() + " for " + delay + "ms");
					m_activeDelays.put(motive.getClass(),
							currentSystemTime + delay);
				}

			}
			// else if the delay has been created and now exceeded
			else if (target < currentSystemTime ) {
				m_component.log("surfacing " + motive.getClass()
						+ " after delay of " + delay + "ms");
				priority = MotivePriority.NORMAL;
				m_activeDelays.remove(motive.getClass());
				m_postDelays.put(motive.getClass(),currentSystemTime
						+ getPostDelayWindow(motive.getClass()));
			} else {
				m_component.log("continuing delay of " + motive.getClass()
						+ " for another " + (target - currentSystemTime) + "ms");
			}
		}
		return priority;
	}

	private int getPostDelayWindow(Class<? extends Motive> _mtvCls) {
		Integer window = m_postDelayWindowMap.get(_mtvCls);
		if (window == null) {
			return 0;
		} else {
			return window;
		}
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

		int postDialogueDelay = 25000;

		m_delayMap.put(LearnObjectFeatureMotive.class, postDialogueDelay);

		m_postDelayWindowMap.put(LearnObjectFeatureMotive.class,
				postDialogueDelay);

		m_delayMap.put(RobotNonSituatedMotive.class, postDialogueDelay);

		// No delays here as the generator delays anyway
		// m_delayMap.put(LookAtViewConeMotive.class, postDialogueDelay);
		// m_postDelayWindowMap.put(LookAtViewConeMotive.class,
		// postDialogueDelay);

	}

}
