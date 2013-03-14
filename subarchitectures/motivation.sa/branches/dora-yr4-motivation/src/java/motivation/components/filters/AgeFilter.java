/**
 * 
 */
package motivation.components.filters;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.CASTTimeUtil;

/**
 * @author marc
 * 
 */
public class AgeFilter implements MotiveFilter {
	private static final long MAX_AGE_MILLIS = 6000;

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
	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		if (checkAge(motive))
			return MotivePriority.HIGH;
		else
			return MotivePriority.UNSURFACE;
	}


	boolean checkAge(Motive motive) {
		CASTTime time = CASTUtils.getTimeServer().getCASTTime();
		return (CASTTimeUtil.diff(time, motive.created) < MAX_AGE_MILLIS);
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
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
