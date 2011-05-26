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
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 * 
 */
public class PlaceMatchingFunction implements ContentMatchingFunction<dBelief> {
	/** the identifier of the PlaceId attribute */
	public static final String PLACE_ID = "PlaceId";

	private long placeId;

	/**
	 * @param placeId
	 */
	public PlaceMatchingFunction(long placeId) {
		this.placeId = placeId;
	}

	@Override
	public boolean matches(dBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(SpatialData.Place.class))) {
			assert (r.content instanceof CondIndependentDistribs);

			IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
					.create(dBelief.class, r);
			FormulaDistribution fv = b.getContent().get(PLACE_ID);
			Formula mostLikelyPlace = fv.getDistribution().getMostLikely();
			int idVal = mostLikelyPlace.getInteger();
			return idVal == placeId;

		} else {
			return false;
		}
	}

}
