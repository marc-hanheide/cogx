/**
 * 
 */
package motivation.components.filters;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import cast.architecture.ManagedComponent;

/**
 * @author marc
 * 
 */
public class LimitTriesFilter implements MotiveFilter {
	private static final long MAX_TRIES = 3;
	
	ManagedComponent component;
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
	public MotivePriority shouldBeSurfaced(Motive motive) {
		if (motive.tries<=MAX_TRIES) 
			return MotivePriority.HIGH;
		return MotivePriority.UNSURFACE;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.filters.Filter#shouldBeUnsurfaced(motivation.slice
	 * .Motive)
	 */
	public boolean shouldBeUnsurfaced(Motive motive) {
		return (motive.tries>3);
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
		component = motiveFilterManager;
		
	}

}
