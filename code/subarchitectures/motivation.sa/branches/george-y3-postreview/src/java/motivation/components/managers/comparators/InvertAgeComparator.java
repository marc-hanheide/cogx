package motivation.components.managers.comparators;

import java.util.Comparator;

import motivation.slice.Motive;

public class InvertAgeComparator implements Comparator<Motive> {
	@Override
	public int compare(Motive arg0, Motive arg1) {
		return - (new CASTTimeComparator().compare(arg0.created,arg1.created));
	}

}
