/**
 * 
 */
package motivation.components.filters;

import motivation.slice.Motive;
import cast.architecture.ManagedComponent;

/**
 * @author marc
 * 
 */
public class LimitTriesFilter implements MotiveFilter {
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
	public boolean shouldBeSurfaced(Motive motive) {
		return (motive.tries<=3);
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
