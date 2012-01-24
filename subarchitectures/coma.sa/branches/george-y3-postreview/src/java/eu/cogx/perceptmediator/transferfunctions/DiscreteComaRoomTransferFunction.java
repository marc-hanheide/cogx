/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import SpatialProbabilities.JointProbabilityValue;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import eu.cogx.beliefs.slice.GroundedBelief;

/**
 * @author marc
 * 
 */
public class DiscreteComaRoomTransferFunction extends ComaRoomTransferFunction {

	public DiscreteComaRoomTransferFunction(ManagedComponent component,
			double hasPersonProbability) {
		super(component, hasPersonProbability);
	}

	@Override
	protected void fillBelief(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief,
			WorkingMemoryChange wmc, ComaRoom from) {
		super.fillBelief(belief, wmc, from);
		if (from.categories.massFunction == null) {
			component.getLogger().info(
					"Coma room without a category yet, not mediating!");
			return;
		}

		IndependentFormulaDistributions distr = belief.getContent();
		FormulaDistribution fd = FormulaDistribution.create();
		double maxProb = 0.0;
		String maxLabel = null;
		for (JointProbabilityValue jp : from.categories.massFunction) {
			if (jp.probability > maxProb) {
				maxProb = jp.probability;
				maxLabel = ((SpatialProbabilities.StringRandomVariableValue) (jp.variableValues[0])).value;
			}
		}
		if (maxLabel != null) {
			fd.add(maxLabel, 1.0);

		} else {
			fd.add("unknown", 1.0);
		}
		distr.put(CATEGORY_ID, fd);
	}

}
