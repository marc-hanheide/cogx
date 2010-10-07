package eu.cogx.perceptmediator.dora;

import VisionData.VisualObject;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class VisualObjectMatchingFunction implements
		ContentMatchingFunction<dBelief> {

	private String objectId;

	public VisualObjectMatchingFunction(String id) {
		objectId = id;
	}

	@Override
	public boolean matches(dBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class))) {
			IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
					.create(dBelief.class, r);
			FormulaDistribution fv = b.getContent().get("ObjectId");
			Formula mostLikelyPlace = fv.getDistribution().getMostLikely();
			return mostLikelyPlace.getProposition().equals(objectId);
		} else {
			return false;
		}
	}

}
