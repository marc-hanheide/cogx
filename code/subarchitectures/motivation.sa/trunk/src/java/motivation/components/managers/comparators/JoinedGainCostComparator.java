package motivation.components.managers.comparators;

import java.util.Comparator;

import motivation.slice.Motive;

public class JoinedGainCostComparator implements Comparator<Motive> {

	/**
	 * the lower the costs the "smaller" should the Comparator
	 * consider the motive for sorting
	 * 
	 */
	@Override
	public int compare(Motive arg0, Motive arg1) {
		double compareValue0 = arg0.informationGain / (arg0.costs*2);
		double compareValue1 = arg1.informationGain / (arg1.costs*2);
		
		if (compareValue0 < compareValue1)
			return 1;
		else if (compareValue0 > compareValue1)
			return -1;
		else
			return 0;
	}

}
