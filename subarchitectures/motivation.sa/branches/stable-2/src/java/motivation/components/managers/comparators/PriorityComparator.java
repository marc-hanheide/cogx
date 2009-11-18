package motivation.components.managers.comparators;

import java.util.Comparator;

import motivation.slice.Motive;

public class PriorityComparator implements Comparator<Motive> {

	/**
	 * the higher the priority the "smaller" should the Comparator
	 * consider the motive for sorting
	 * 
	 */
	@Override
	public int compare(Motive arg0, Motive arg1) {
		if (arg0.priority.value() < arg1.priority.value())
			return 1;
		else if (arg0.priority.value() > arg1.priority.value())
			return -1;
		else
			return 0;
	}

}
