/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaDistributionsFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentFormulaDistributions> {

	@Override
	public IndependentFormulaDistributions create(ProbDistribution pd) {
		return IndependentFormulaDistributions.create(pd);
	}

}
