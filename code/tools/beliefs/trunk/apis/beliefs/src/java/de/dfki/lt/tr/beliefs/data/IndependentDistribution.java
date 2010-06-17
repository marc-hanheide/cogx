/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import java.util.HashMap;

import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistribution;
import de.dfki.lt.tr.beliefs.factories.DistributionFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentDistribution extends
		GenericIndependentDistribution<Distribution<ProbDistribution>> {

	public static IndependentDistribution create() {
		return create(new CondIndependentDistribs(new HashMap<String, ProbDistribution>()));
	}

	public static IndependentDistribution create(ProbDistribution pd) {
		return new IndependentDistribution(pd);
	}

	protected IndependentDistribution(ProbDistribution content) {
		super(DistributionFactory.get(), content);
	}

}
