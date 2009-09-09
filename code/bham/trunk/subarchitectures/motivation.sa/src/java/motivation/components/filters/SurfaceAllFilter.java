/**
 * 
 */
package motivation.components.filters;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.CASTTimeHelper;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import motivation.slice.Motive;
import motivation.util.CASTTimeUtil;
import motivation.util.WMMotiveSet;

/**
 * @author marc
 * 
 */
public class SurfaceAllFilter extends Filter {

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
