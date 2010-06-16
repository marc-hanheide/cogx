/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import java.util.HashMap;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.genericproxies.Content;
import de.dfki.lt.tr.beliefs.factories.ContentFactory;
import de.dfki.lt.tr.beliefs.factories.IndependentDistributionFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.framing.TemporalInterval;
import de.dfki.lt.tr.beliefs.slice.history.AbstractBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentDistributionBelief<T extends dBelief> extends GenericBelief<T, IndependentDistribution> {

	public static <T2 extends dBelief> IndependentDistributionBelief<T2> create(
			Class<? extends T2> type, Ice.Object o) {
		return new IndependentDistributionBelief<T2>(type, o);
	}

	public static <T2 extends dBelief> IndependentDistributionBelief<T2> create(
			Class<? extends T2> type) {
		T2 internal =createDefault(type);
		internal.content = new CondIndependentDistribs(new HashMap<String, ProbDistribution>());
		return new IndependentDistributionBelief<T2>(type, internal);
	}

	protected IndependentDistributionBelief(Class<? extends T> class1, Ice.Object content) {
		super(class1, new IndependentDistributionFactory(), content);
	}

}
