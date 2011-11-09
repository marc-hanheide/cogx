/**
 * 
 */
package motivation.components.filters;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;

import org.apache.log4j.Logger;

import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class LimitTriesFilter implements MotiveFilter {
	private static final long DEFAULT_MAX_TRIES = 10;
	private long maxTries = DEFAULT_MAX_TRIES;

	/**
	 * @param specificType
	 */
	public LimitTriesFilter() {
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
		if (motive.tries <= maxTries)
			return MotivePriority.HIGH;
		return MotivePriority.UNSURFACE;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
	}

	@Override
	public void start() {
	}

	@Override
	public void configure(Map<String, String> arg0) {
		String arg = arg0.get("--max-tries");
		if (arg != null) {
			maxTries = Integer.parseInt(arg);
		}
		Logger.getLogger(GainFilter.class).info(
				"configure: maxTries=" + maxTries);

	}

}
