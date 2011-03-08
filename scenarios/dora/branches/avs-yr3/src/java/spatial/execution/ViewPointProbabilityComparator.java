package spatial.execution;

import java.util.Comparator;

import SpatialData.ViewPoint;

/**
 * Sorts viewpoits by their probability.
 * 
 * @author nah
 * 
 */
public class ViewPointProbabilityComparator implements Comparator<ViewPoint> {

	private static ViewPointProbabilityComparator m_comparator;

	@Override
	public int compare(ViewPoint _o1, ViewPoint _o2) {
		return Math
				.round((float) ((100 * _o2.probability) - (100 * _o1.probability)));
	}

	public static final Comparator<? super ViewPoint> getInstance() {
		if (m_comparator == null) {
			m_comparator = new ViewPointProbabilityComparator();
		}
		return m_comparator;
	}

}
