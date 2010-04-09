package castutils.viewer.plugins;

import java.util.Vector;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.distribs.ProbDistribution;

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
//			if (belief.frame != null) {
//				extraInfo.add("ST frame:" + belief.frame.getClass().getSimpleName());
//			}
			extraInfo.add("Type: " + belief.getClass().getSimpleName());
			if (belief.content instanceof CondIndependentDistribs) {
				CondIndependentDistribs dist = (CondIndependentDistribs) belief.content;
				for (ProbDistribution pd : dist.distribs) {
					if (pd instanceof FeatureValueDistribution) {
						FeatureValueDistribution fvd = (FeatureValueDistribution) pd;
						extraInfo.add(fvd.feat.toString());
					}
				}
			}
		}
		return extraInfo;
	}

}
