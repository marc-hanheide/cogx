package motivation.components.managers.comparators;

/**
 * a cascaded comparator implementing the following sorting criteria:
 * <ol>
 * <li>Motive priority</li>
 * <li>Motive information gain</li>
 * <li>Motive age (inverted)</li>
 * </ol>
 * 
 * @author marc
 * 
 */
public class PriorityTriesCostGainComparator extends CascadedComparator {

	/**
	 * 
	 */
	public PriorityTriesCostGainComparator() {
		super();
		super.addComparator(new PriorityComparator());
		super.addComparator(new TriesComparator());
		super.addComparator(new JoinedGainCostComparator());
		super.addComparator(new InvertAgeComparator());
	}

}
