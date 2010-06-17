/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories.specific;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaDistributionsFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentFormulaDistributions> {

	private static IndependentFormulaDistributionsFactory singleton = null;

	public static IndependentFormulaDistributionsFactory get() {
		if (singleton == null)
			singleton = new IndependentFormulaDistributionsFactory();
		return singleton;
	}

	@Override
	public IndependentFormulaDistributions create(ProbDistribution pd) {
		return IndependentFormulaDistributions.create(pd);
	}

}
