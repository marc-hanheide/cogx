package binder.perceptmediator.attic;

import comadata.ComaRoom;

import VisionData.VisualObject;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.featurecontent.StringValue;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

public class ComaRoomMatchingFunction implements
		ContentMatchingFunction<PerceptBelief> {

	private String objectId;

	public ComaRoomMatchingFunction(String id) {
		objectId=id;
	}

	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction.getBeliefTypeFromCastType(ComaRoom.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			CondIndependentDistribs dist = (CondIndependentDistribs) r.content;
			BasicProbDistribution fv = (BasicProbDistribution) dist.distribs.get("RoomId");
			StringValue idVal = (StringValue) ((FeatureValues)fv.values).values.get(0).val;
			return idVal.val.equals(objectId);
		}
		else {
			return false;
		}
	}

}
