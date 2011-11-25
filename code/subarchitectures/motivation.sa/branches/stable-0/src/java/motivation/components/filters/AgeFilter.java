/**
 * 
 */
package motivation.components.filters;

import motivation.slice.Motive;
import motivation.util.CASTTimeUtil;
import motivation.components.filters.AbstractFilter;

/**
 * @author marc
 * 
 */
public class AgeFilter extends AbstractFilter {

	/**
	 * @param specificType
	 */
	public  AgeFilter() {
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