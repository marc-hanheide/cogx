/**
 * 
 */
package motivation.components.filters;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class PassAllFilter implements MotiveFilter {

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.filters.Filter#shouldBeSurfaced(motivation.slice
	 * .Motive)
	 */
	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		return MotivePriority.HIGH;
	}

	@Override
	public void setManager(MotiveFilterManager motiveFilterManager) {
	}

	@Override
	public void start() {
	}

	@Override
	public void configure(Map<String, String> arg0) {
	}

}
