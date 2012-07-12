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
public class PrioritizeExploreCategorizeFilter implements MotiveFilter {

	/* (non-Javadoc)
	 * @see motivation.components.filters.MotiveFilter#checkMotive(motivation.slice.Motive, cast.cdl.WorkingMemoryChange)
	 */
	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		// TODO Auto-generated method stub
		return null;
	}

	/* (non-Javadoc)
	 * @see motivation.components.filters.MotiveFilter#setManager(motivation.components.filters.MotiveFilterManager)
	 */
	@Override
	public void setManager(MotiveFilterManager motiveFilterManager) {
		// TODO Auto-generated method stub

	}

	@Override
	public void start() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void configure(Map<String, String> arg0) {
		// TODO Auto-generated method stub
		
	}

}
