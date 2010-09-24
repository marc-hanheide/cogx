/**
 * 
 */
package binder.perceptmediator.transferfunctions.helpers;

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.featurecontent.IntegerValue;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

/**
 * @author marc
 *
 */
public class AgentMatchingFunction implements
		ContentMatchingFunction<PerceptBelief> {

	long agentId;
	public AgentMatchingFunction(long agentID) {
		this.agentId = agentID;
	}

	/* (non-Javadoc)
	 * @see castutils.castextensions.WMContentWaiter.ContentMatchingFunction#matches(Ice.ObjectImpl)
	 */
	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals("Robot")) {
			assert (r.content instanceof CondIndependentDistribs);
			CondIndependentDistribs dist = (CondIndependentDistribs) r.content;
			BasicProbDistribution fv = (BasicProbDistribution) dist.distribs.get("AgentId");
			IntegerValue idVal = (IntegerValue) ((FeatureValues)fv.values).values.get(0).val;
			return idVal.val == agentId;
			
		}
		else {
			return false;
		}
	}
}
