package motivation.components.managers.comparators;


/**
 * a cascaded comparator implementing the following sorting criteria:
 * <ol>
 * <li>Motive priority</li>
 * <li>Motive information gain</li>
 * <li>Motive age (inverted)</li>
 * </ol>
 * @author marc
 *
 */
public class PriorityGainComparator extends CascadedComparator {

	/**
	 * 
	 */
	public PriorityGainComparator() {
		super();
		super.addComparator(new PriorityComparator());
		super.addComparator(new InformationGainComparator());
		super.addComparator(new InvertAgeComparator());
	}


}
