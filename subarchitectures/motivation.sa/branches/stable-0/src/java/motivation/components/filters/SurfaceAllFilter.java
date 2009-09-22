/**
 * 
 */
package motivation.components.filters;

import motivation.slice.Motive;

/**
 * @author marc
 * 
 */
public class SurfaceAllFilter extends AbstractFilter {

	/**
	 * @param specificType
	 */
	public SurfaceAllFilter() {
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
		return true;
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
		return false;
	}

}
