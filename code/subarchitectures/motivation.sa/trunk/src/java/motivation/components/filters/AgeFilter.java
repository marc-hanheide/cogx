/**
 * 
 */
package motivation.components.filters;

import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;
import cast.core.CASTUtils;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.util.CASTTimeUtil;
import motivation.components.filters.MotiveFilterManager;

/**
 * @author marc
 * 
 */
public class AgeFilter implements MotiveFilter {
	private static final long MAX_AGE_MILLIS = 6000;
	ManagedComponent component;

	/**
	 * @param specificType
	 */
	public AgeFilter() {
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
		if (checkAge(motive))
			return MotivePriority.HIGH;
		else
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
		if (checkAge(motive))
			return false;
		component.log("motive was too old... unsurfacing it!");
		return true;
	}

	boolean checkAge(Motive motive) {
		CASTTime time = CASTUtils.getTimeServer().getCASTTime();
		return (CASTTimeUtil.diff(time, motive.created) < MAX_AGE_MILLIS);
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
		component = motiveFilterManager;

	}

}
