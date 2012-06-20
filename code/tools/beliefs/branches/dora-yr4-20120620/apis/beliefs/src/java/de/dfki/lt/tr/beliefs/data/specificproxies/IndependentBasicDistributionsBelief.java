/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import java.util.HashMap;

import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBelief;
import de.dfki.lt.tr.beliefs.factories.specific.IndependentBasicDistributionsFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentBasicDistributionsBelief<T extends dBelief> extends
		GenericBelief<T, IndependentBasicDistributions> {

	public static <T2 extends dBelief> IndependentBasicDistributionsBelief<T2> create(
			Class<? extends T2> type, dBelief o) {
		return new IndependentBasicDistributionsBelief<T2>(type, o);
	}

	public static <T2 extends dBelief> IndependentBasicDistributionsBelief<T2> create(
			Class<? extends T2> type) {
		T2 internal = createDefault(type);
		internal.content = new CondIndependentDistribs(
				new HashMap<String, ProbDistribution>());
		return create(type, internal);
	}

	protected IndependentBasicDistributionsBelief(Class<? extends T> class1,
			dBelief content) {
		super(class1, IndependentBasicDistributionsFactory.get(), content);
	}

}
