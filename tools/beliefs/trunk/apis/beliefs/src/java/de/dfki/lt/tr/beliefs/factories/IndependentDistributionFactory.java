/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentDistributionFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentDistribution> {

	private static IndependentDistributionFactory singleton = null;

	public static IndependentDistributionFactory get() {
		if (singleton == null)
			singleton = new IndependentDistributionFactory();
		return singleton;
	}

	@Override
	public IndependentDistribution create(ProbDistribution pd) {
		return IndependentDistribution.create(pd);
	}

}
