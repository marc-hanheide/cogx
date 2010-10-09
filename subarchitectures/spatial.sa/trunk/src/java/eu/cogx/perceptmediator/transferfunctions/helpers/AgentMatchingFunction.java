/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.helpers;

import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.GroundedBelief;

/**
 * @author marc
 * 
 */
public class AgentMatchingFunction implements
		ContentMatchingFunction<dBelief> {

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
	public boolean matches(dBelief r) {
		if (r.type.equals("Robot")) {
			assert (r.content instanceof CondIndependentDistribs);
			IndependentFormulaDistributionsBelief<GroundedBelief> b = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, r);
			FormulaDistribution fv = b.getContent().get("AgentId");
			Formula mostLikelyPlace = fv.getDistribution().getMostLikely();
			int idVal = mostLikelyPlace.getInteger();
			return idVal == agentId;

		} else {
			return false;
		}
	}
}
