/**
 * 
 */
package motivation.components.filters;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.RobotInitiativeMotive;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.CASTTimeUtil;

/**
 * @author marc
 * 
 */
public class MinAgeFilter implements MotiveFilter {

	private static final long MIN_AGE_MILLIS = 6000;
	private long minAgeMillis=MIN_AGE_MILLIS;

	/**
	 * @param specificType
	 */
	public MinAgeFilter() {
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
		if (motive instanceof RobotInitiativeMotive) {
		CASTTime time = CASTUtils.getTimeServer().getCASTTime();
		return (CASTTimeUtil.diff(time, motive.created) > minAgeMillis);
		} else
			return true;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
	}

	@Override
	public void start() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void configure(Map<String, String> arg0) {
		String arg = arg0.get("--min-age");
		if (arg != null) {
			minAgeMillis = Long.parseLong(arg);			
		}
		
	}

}
