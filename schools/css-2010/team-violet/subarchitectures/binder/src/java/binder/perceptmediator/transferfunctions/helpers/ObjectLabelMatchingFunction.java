package binder.perceptmediator.transferfunctions.helpers;

import VisionData.VisualObject;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.featurecontent.StringValue;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

public class ObjectLabelMatchingFunction implements
		ContentMatchingFunction<PerceptBelief> {

	private String label;

	public ObjectLabelMatchingFunction(String _label) {
		label=_label;
	}

	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction.getBeliefTypeFromCastType(VisualObject.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			CondIndependentDistribs dist = (CondIndependentDistribs) r.content;
			BasicProbDistribution fv = (BasicProbDistribution) dist.distribs.get("label");
			StringValue labelVal = (StringValue) ((FeatureValues)fv.values).values.get(0).val;
			return labelVal.val.equals(label);
		}
		else {
			return false;
		}
	}

}
