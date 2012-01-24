/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package castutils.castextensions;

import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class BeliefWaiter<T extends dBelief> extends WMContentWaiter<T> {

	/**
	 * @param view
	 */
	public BeliefWaiter(WMView<T> view) {
		super(view);
	}

}
