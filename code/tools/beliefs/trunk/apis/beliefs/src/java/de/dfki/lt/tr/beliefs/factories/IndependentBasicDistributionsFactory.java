/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentBasicDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentBasicDistributionsFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentBasicDistributions> {

	@Override
	public IndependentBasicDistributions create(ProbDistribution pd) {
		return IndependentBasicDistributions.create(pd);
	}

}
