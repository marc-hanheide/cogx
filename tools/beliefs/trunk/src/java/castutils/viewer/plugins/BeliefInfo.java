package castutils.viewer.plugins;

import java.util.Vector;
import java.util.Map.Entry;

import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import castutils.viewer.plugins.Plugin;
import de.dfki.lt.tr.beliefs.data.Belief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

//import binder.utils.BeliefModelUtils;

/**
 * @author Geert-Jan Kruijff
 * @version 091016
 */

public class BeliefInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		Belief<dBelief> belief = Belief.create(dBelief.class, (dBelief) iceObject);
			String agentStatus = belief.getStatus().getClass().getSimpleName();
			extraInfo.add("Status: " + agentStatus);
			extraInfo.add("Type: " + belief.getType());
//			String offsprings = "";
//			for (WorkingMemoryAddress offspringWMA : ((CASTBeliefHistory)belief.hist).offspring) {
//				offsprings+=" "+offspringWMA.id;
//			}
//			extraInfo.add("offspring:" + offsprings);
//			if (belief.content instanceof CondIndependentDistribs) {
//				CondIndependentDistribs dist = (CondIndependentDistribs) belief.content;
//				String features="";
//				for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
//					if (pd.getValue() instanceof BasicProbDistribution) {
//						BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
//						features+=pd.getKey()+"=[";
//						for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
//							String featStr = toString(fv.val);
//							features+=featStr+" ";
//						}
//						features+="] ";
//					}
//				}
//				extraInfo.add("Features: " + features);
//			}
//		}
		return extraInfo;
	}

//	/**
//	 * @param fv
//	 * @return
//	 */
//	public static String toString(FeatureValue fv) {
//		String featStr="*";
//		if (fv instanceof IntegerValue)
//			featStr=Integer.toString(((IntegerValue) fv).val);
//		if (fv instanceof PointerValue)
//			featStr=CASTUtils.toString(((PointerValue) fv).beliefId);
//		if (fv instanceof StringValue)
//			featStr=((StringValue) fv).val;
//		if (fv instanceof FloatValue)
//			featStr= Double.toString(((FloatValue) fv).val);
//		if (fv instanceof BooleanValue)
//			featStr=Boolean.toString(((BooleanValue) fv).val);
//		return featStr;
//	}

}
