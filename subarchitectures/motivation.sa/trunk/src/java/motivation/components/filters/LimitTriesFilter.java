/**
 * 
 */
package motivation.components.filters;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class LimitTriesFilter implements MotiveFilter {
	private static final long MAX_TRIES = 10;
	
	/**
	 * @param specificType
	 */
	public  LimitTriesFilter() {
		super();
		// TODO Auto-generated constructor stub
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.filters.Filter#shouldBeSurfaced(motivation.slice
	 * .Motive)
	 */
	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		if (motive.tries<=MAX_TRIES) 
			return MotivePriority.HIGH;
		return MotivePriority.UNSURFACE;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
	}

	@Override
	public void start() {
	}

}
