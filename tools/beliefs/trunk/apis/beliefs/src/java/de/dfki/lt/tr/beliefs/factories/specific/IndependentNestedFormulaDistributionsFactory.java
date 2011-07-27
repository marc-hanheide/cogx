/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories.specific;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentNestedFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentNestedFormulaDistributionsFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentNestedFormulaDistributions> {

	private static IndependentNestedFormulaDistributionsFactory singleton = null;

	public static IndependentNestedFormulaDistributionsFactory get() {
		if (singleton == null)
			singleton = new IndependentNestedFormulaDistributionsFactory();
		return singleton;
	}

	@Override
	public IndependentNestedFormulaDistributions create(ProbDistribution pd) {
		return IndependentNestedFormulaDistributions.create(pd);
	}

}
