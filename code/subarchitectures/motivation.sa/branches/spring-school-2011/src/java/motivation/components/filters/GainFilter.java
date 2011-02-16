/**
 * 
 */
package motivation.components.filters;

import java.util.Map;

import org.apache.log4j.Logger;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class GainFilter implements MotiveFilter {

	public static final double DEFAULT_MIN_GAIN = 0.3;
	private double minGain = DEFAULT_MIN_GAIN;

	/**
	 * @param specificType
	 */
	public GainFilter() {
		super();
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
		if (motive.informationGain >= minGain)
			return MotivePriority.HIGH;
		else
			return MotivePriority.UNSURFACE;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
	}

	@Override
	public void start() {

	}

	@Override
	public void configure(Map<String, String> arg0) {
		String arg = arg0.get("--min-gain");
		if (arg != null) {
			minGain = Double.parseDouble(arg);			
		}
		Logger.getLogger(GainFilter.class).info("configure: minGain=" + minGain);

	}

}
