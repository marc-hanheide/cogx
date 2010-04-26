/**
 * 
 */
package binder.perceptmediator.transferfunctions.helpers;

import VisionData.VisualObject;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.featurecontent.StringValue;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

/**
 * @author marc
 *
 */
public class VisualObjectMatchingFunction implements ContentMatchingFunction<PerceptBelief> {
	/** the identifier of the PlaceId attribute */ 
	public static final String PLACE_ID = "PlaceId";

	private String personId;
	
	/**
	 * @param personId
	 */
	public VisualObjectMatchingFunction(String personId) {
		this.personId = personId;
	}


	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals(CASTUtils.typeName(VisualObject.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			CondIndependentDistribs dist = (CondIndependentDistribs) r.content;
			BasicProbDistribution fv = (BasicProbDistribution) dist.distribs.get("VisualObjectId");
			StringValue idVal = (StringValue) ((FeatureValues)fv.values).values.get(0).val;
			return idVal.val.equals(personId);
		}
		else {
			return false;
		}
	}
	
}
