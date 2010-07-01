/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.helpers;

import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;

/**
 * @author marc
 *
 */
public class PlaceMatchingFunction implements ContentMatchingFunction<PerceptBelief> {
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
	public boolean matches(PerceptBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction.getBeliefTypeFromCastType(SpatialData.Place.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			
			IndependentFormulaDistributionsBelief<PerceptBelief> b = IndependentFormulaDistributionsBelief.create(PerceptBelief.class, r);
			FormulaDistribution fv = b.getContent().get(PLACE_ID);
			Formula mostLikelyPlace=fv.getDistribution().getMostLikely();
			int idVal = mostLikelyPlace.getInteger();
			return idVal == placeId;
			
		}
		else {
			return false;
		}
	}
	
}
