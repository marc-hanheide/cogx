package motivation.components.managers.comparators;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import motivation.slice.Motive;

/**
 * provides a cascaded Comparator implementation. It's a container for several
 * {@link Comparator} objects that are called in order. The first comparator add
 * with addComparator is checked first. If it return 0 (== objects equal) the
 * next Comparator in the list is checked and so on. Hence, compare iterates on
 * all registered Comparators until one return something not 0.
 * 
 * @author marc
 * 
 */
public class CascadedComparator implements Comparator<Motive> {

	List<Comparator<Motive>> comparators;

	/**
	 * @param comparators
	 */
	public CascadedComparator(List<Comparator<Motive>> comparators) {
		this.comparators = new LinkedList<Comparator<Motive>>(comparators);
	}

	/**
	 * @param comparators
	 */
	public CascadedComparator() {
		this.comparators = new LinkedList<Comparator<Motive>>();
	}

	public void addComparator(Comparator<Motive> comparator) {
		comparators.add(comparator);
	}

	public void removeComparator(Comparator<Motive> comparator) {
		comparators.remove(comparator);
	}

	public void clear() {
		comparators.clear();
	}

	/**
	 * the higher the information gain the "smaller" should the Comparator
	 * consider the motive for sorting
	 * 
	 */
	@Override
	public int compare(Motive arg0, Motive arg1) {
		int result = 0;
		for (Comparator<Motive> c : comparators) {
			result = c.compare(arg0, arg1);
			if (result != 0) { // if we have a "non-equals" result we take this
				// one and break
				break;
			}
		}
		return result;
	}

}
