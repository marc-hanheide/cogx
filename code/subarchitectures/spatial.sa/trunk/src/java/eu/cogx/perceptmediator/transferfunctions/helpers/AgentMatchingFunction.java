/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.helpers;

import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import eu.cogx.beliefs.slice.PerceptBelief;

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

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * castutils.castextensions.WMContentWaiter.ContentMatchingFunction#matches
	 * (Ice.ObjectImpl)
	 */
	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals("Robot")) {
			assert (r.content instanceof CondIndependentDistribs);
			IndependentFormulaDistributionsBelief<PerceptBelief> b = IndependentFormulaDistributionsBelief
					.create(PerceptBelief.class, r);
			FormulaDistribution fv = b.getContent().get("AgentId");
			Formula mostLikelyPlace = fv.getDistribution().getMostLikely();
			int idVal = mostLikelyPlace.getInteger();
			return idVal == agentId;

		} else {
			return false;
		}
	}
}
