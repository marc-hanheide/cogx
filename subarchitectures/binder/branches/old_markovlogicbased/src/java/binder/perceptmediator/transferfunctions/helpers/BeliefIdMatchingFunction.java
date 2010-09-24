/**
 * 
 */
package binder.perceptmediator.transferfunctions.helpers;

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.featurecontent.IntegerValue;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

/**
 * @author marc
 *
 */
public class BeliefIdMatchingFunction implements ContentMatchingFunction<PerceptBelief> {
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
