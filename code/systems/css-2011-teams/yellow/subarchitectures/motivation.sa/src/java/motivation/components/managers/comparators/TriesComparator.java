package motivation.components.managers.comparators;

import java.util.Comparator;

import motivation.slice.Motive;

public class TriesComparator implements Comparator<Motive> {
	@Override
	public int compare(Motive arg0, Motive arg1) {
		if (arg0.tries>arg1.tries) {
			return 1;
		} else if (arg0.tries<arg1.tries)
			return -1;
		else {
			return 0;
		}

	}

}
