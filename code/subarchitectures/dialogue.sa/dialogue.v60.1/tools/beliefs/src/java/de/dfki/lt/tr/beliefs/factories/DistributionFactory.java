/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class DistributionFactory extends AbstractProxyFactory<ProbDistribution, Distribution<ProbDistribution>> {

	private static DistributionFactory singleton = null;

	public static DistributionFactory get() {
		if (singleton == null)
			singleton = new DistributionFactory();
		return singleton;
	}
	
	@Override
	public Distribution<ProbDistribution> create(ProbDistribution pd) {
		return Distribution.create(ProbDistribution.class, pd);
	}

}
