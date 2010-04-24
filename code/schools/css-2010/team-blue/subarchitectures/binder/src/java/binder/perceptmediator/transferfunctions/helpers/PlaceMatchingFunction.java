/**
 * 
 */
package binder.perceptmediator.transferfunctions.helpers;

import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.featurecontent.IntegerValue;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

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
		if (r.type.equals(CASTUtils.typeName(SpatialData.Place.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			CondIndependentDistribs dist = (CondIndependentDistribs) r.content;
			BasicProbDistribution fv = (BasicProbDistribution) dist.distribs.get(PLACE_ID);
			IntegerValue idVal = (IntegerValue) ((FeatureValues)fv.values).values.get(0).val;
			return idVal.val == placeId;
			
		}
		else {
			return false;
		}
	}
	
}
