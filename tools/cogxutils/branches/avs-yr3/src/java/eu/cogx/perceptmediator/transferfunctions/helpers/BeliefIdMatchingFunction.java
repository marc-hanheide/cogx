/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.helpers;

import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import eu.cogx.beliefs.slice.PerceptBelief;

/**
 * @author marc
 * 
 */
public class BeliefIdMatchingFunction implements
		ContentMatchingFunction<PerceptBelief> {
	/** the identifier of the PlaceId attribute */

	private String beliefId;

	/**
	 * @param beliefId
	 */
	public BeliefIdMatchingFunction(String beliefId) {
		this.beliefId = beliefId;
	}

	@Override
	public boolean matches(PerceptBelief r) {
		return r.id.equals(beliefId);
	}

}
