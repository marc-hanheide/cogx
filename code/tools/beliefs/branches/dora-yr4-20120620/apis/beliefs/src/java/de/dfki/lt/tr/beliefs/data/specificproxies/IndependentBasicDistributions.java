/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import de.dfki.lt.tr.beliefs.data.BasicDistribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistribution;
import de.dfki.lt.tr.beliefs.factories.BasicDistributionFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentBasicDistributions extends
		GenericIndependentDistribution<BasicDistribution> {

	public static IndependentBasicDistributions create(ProbDistribution o) {
		return new IndependentBasicDistributions(o);
	}

	protected IndependentBasicDistributions(ProbDistribution content) {
		super(BasicDistributionFactory.get(), content);
	}

}
