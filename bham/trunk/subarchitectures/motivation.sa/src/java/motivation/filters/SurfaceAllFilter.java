/**
 * 
 */
package motivation.filters;

import motivation.util.WMMotiveSet;

/**
 * @author marc
 *
 */
public class SurfaceAllFilter extends Filter {
	WMMotiveSet motives;

	/**
	 * 
	 */
	public SurfaceAllFilter() {
		super();
		motives = WMMotiveSet.create(this);
	}
	
	
	
}
