package motivation.components.managers.comparators;

import java.util.Comparator;

import motivation.slice.Motive;

public class JoinedGainCostComparator implements Comparator<Motive> {

	
	private static int COST_MULTIPLIER = 5;
	
	/**
	 * the lower the costs the "smaller" should the Comparator
	 * consider the motive for sorting
	 * 
	 */
	@Override
	public int compare(Motive arg0, Motive arg1) {
		double div0=Math.max(1e-3,arg0.costs) * COST_MULTIPLIER;
		double div1=Math.max(1e-3,arg1.costs) * COST_MULTIPLIER;
		double compareValue0 = arg0.informationGain / div0;
		double compareValue1 = arg1.informationGain / div1;
		
		if (compareValue0 < compareValue1)
			return 1;
		else if (compareValue0 > compareValue1)
			return -1;
		else
			return 0;
	}

}
