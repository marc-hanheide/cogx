package castutils.viewer.plugins;

import java.util.Vector;
import java.util.Map.Entry;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.FloatValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.featurecontent.StringValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;

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
			String offsprings = "";
			for (WorkingMemoryAddress offspringWMA : ((CASTBeliefHistory)belief.hist).offspring) {
				offsprings+=" "+offspringWMA.id;
			}
			extraInfo.add("offspring:" + offsprings);
			if (belief.content instanceof CondIndependentDistribs) {
				CondIndependentDistribs dist = (CondIndependentDistribs) belief.content;
				String features="";
				for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
					if (pd.getValue() instanceof BasicProbDistribution) {
						BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
						features+=pd.getKey()+"=[";
						for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
							String featStr = toString(fv.val);
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

	/**
	 * @param fv
	 * @return
	 */
	public static String toString(FeatureValue fv) {
		String featStr="*";
		if (fv instanceof IntegerValue)
			featStr=Integer.toString(((IntegerValue) fv).val);
		if (fv instanceof PointerValue)
			featStr=CASTUtils.toString(((PointerValue) fv).beliefId);
		if (fv instanceof StringValue)
			featStr=((StringValue) fv).val;
		if (fv instanceof FloatValue)
			featStr= Double.toString(((FloatValue) fv).val);
		if (fv instanceof BooleanValue)
			featStr=Boolean.toString(((BooleanValue) fv).val);
		return featStr;
	}

}
