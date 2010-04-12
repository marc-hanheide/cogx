package castutils.viewer.plugins;

import java.util.Vector;
import java.util.Map.Entry;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.StringValue;

//import binder.utils.BeliefModelUtils;

/**
 * @author Geert-Jan Kruijff
 * @version 091016
 */

public class BeliefInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		Belief belief = (Belief) iceObject;
		if (belief != null) {
			String agentStatus = belief.estatus.getClass().getSimpleName();
			extraInfo.add("Status: " + agentStatus);
			extraInfo.add("Type: " + belief.type);
			extraInfo.add("Class: " + belief.getClass().getSimpleName());
			if (belief.content instanceof CondIndependentDistribs) {
				CondIndependentDistribs dist = (CondIndependentDistribs) belief.content;
				String features="";
				for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
					if (pd.getValue() instanceof FeatureValueDistribution) {
						FeatureValueDistribution fvd = (FeatureValueDistribution) pd.getValue();
						features+=pd.getKey()+"=[";
						for (FeatureValueProbPair fv : fvd.values) {
							String featStr="*";
							if (fv.val instanceof IntegerValue)
								featStr=Integer.toString(((IntegerValue) fv.val).val);
							if (fv.val instanceof StringValue)
								featStr=((StringValue) fv.val).val;
							features+=featStr+" ";
						}
						features+="] ";
					}
				}
				extraInfo.add("Features: " + features);
			}
		}
		return extraInfo;
	}

}
