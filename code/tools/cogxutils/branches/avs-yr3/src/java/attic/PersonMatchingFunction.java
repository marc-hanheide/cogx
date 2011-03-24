/**
 * 
 */
package binder.perceptmediator.attic;

import VisionData.Person;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.featurecontent.StringValue;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

/**
 * @author marc
 *
 */
public class PersonMatchingFunction implements ContentMatchingFunction<PerceptBelief> {
	/** the identifier of the PlaceId attribute */ 
	public static final String PLACE_ID = "PlaceId";

	private String personId;
	
	/**
	 * @param personId
	 */
	public PersonMatchingFunction(String personId) {
		this.personId = personId;
	}


	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction.getBeliefTypeFromCastType(Person.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			CondIndependentDistribs dist = (CondIndependentDistribs) r.content;
			BasicProbDistribution fv = (BasicProbDistribution) dist.distribs.get("PersonId");
			StringValue idVal = (StringValue) ((FeatureValues)fv.values).values.get(0).val;
			return idVal.val.equals(personId);
		}
		else {
			return false;
		}
	}
	
}
