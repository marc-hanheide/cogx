package eu.cogx.perceptmediator.dora;

import VisionData.VisualObject;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;

import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class VisualObjectMatchingFunction implements
		ContentMatchingFunction<PerceptBelief> {

	private String objectId;

	public VisualObjectMatchingFunction(String id) {
		objectId = id;
	}

	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class))) {
			IndependentFormulaDistributionsBelief<PerceptBelief> b = IndependentFormulaDistributionsBelief
					.create(PerceptBelief.class, r);
			FormulaDistribution fv = b.getContent().get("ObjectId");
			Formula mostLikelyPlace = fv.getDistribution().getMostLikely();
			return mostLikelyPlace.getProposition().equals(objectId);
		} else {
			return false;
		}
	}

}
