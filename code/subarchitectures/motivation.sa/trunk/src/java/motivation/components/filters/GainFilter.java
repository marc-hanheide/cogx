/**
 * 
 */
package motivation.components.filters;

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
public class GainFilter implements MotiveFilter {

	public static final double MIN_GAIN = 0.3;

	/**
	 * @param specificType
	 */
	public GainFilter() {
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
		if (motive.informationGain >= MIN_GAIN)
			return MotivePriority.HIGH;
		else
			return MotivePriority.UNSURFACE;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
	}

	@Override
	public void start() {
		// TODO Auto-generated method stub

	}

}
