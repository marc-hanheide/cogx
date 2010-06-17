/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.BasicDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class BasicDistributionFactory extends
		AbstractProxyFactory<ProbDistribution, BasicDistribution> {

	@Override
	public BasicDistribution create(ProbDistribution pd) {
		return BasicDistribution.create(pd);
	}
}
