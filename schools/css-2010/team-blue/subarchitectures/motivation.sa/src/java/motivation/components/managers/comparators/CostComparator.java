package motivation.components.managers.comparators;

import java.util.Comparator;

import motivation.slice.Motive;

public class CostComparator implements Comparator<Motive> {

	/**
	 * the lower the costs the "smaller" should the Comparator
	 * consider the motive for sorting
	 * 
	 */
	@Override
	public int compare(Motive arg0, Motive arg1) {
		if (arg0.costs < arg1.costs)
			return -1;
		else if (arg0.costs > arg1.costs)
			return 1;
		else
			return 0;
	}

}
