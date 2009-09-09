/**
 * 
 */
package motivation.components.filters;

import motivation.slice.Motive;
import motivation.util.CASTTimeUtil;

/**
 * @author marc
 * 
 */
public class AgeFilter extends Filter {

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.filters.Filter#shouldBeSurfaced(motivation.slice
	 * .Motive)
	 */
	@Override
	protected boolean shouldBeSurfaced(Motive motive) {
		return checkAge(motive);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.filters.Filter#shouldBeUnsurfaced(motivation.slice
	 * .Motive)
	 */
	@Override
	protected boolean shouldBeUnsurfaced(Motive motive) {
		if (checkAge(motive))
			return false;
		log("motive was too old... unsurfacing it!");
		return true;
	}

	boolean checkAge(Motive motive) {
		return (CASTTimeUtil.diff(getCASTTime(), motive.created) < 6000);
	}

}
