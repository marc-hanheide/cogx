package motivation.components.managers.comparators;

import java.util.Comparator;

import cast.cdl.CASTTime;

public class CASTTimeComparator implements Comparator<CASTTime> {

	@Override
	public int compare(CASTTime arg0, CASTTime arg1) {
		if (arg0.s < arg1.s)
			return -1;
		else if (arg0.s > arg1.s)
			return 1;
		else if (arg0.s == arg1.s) {
			if (arg0.us < arg1.us)
				return -1;
			else if (arg0.us > arg1.us)
				return 1;
			else if (arg0.s == arg1.s)
				return 0;
		}
		return 0;

	}

}
