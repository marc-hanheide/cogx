/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class FormulaDistributionFactory extends
		AbstractProxyFactory<ProbDistribution, FormulaDistribution> {

	@Override
	public FormulaDistribution create(ProbDistribution pd) {
		return FormulaDistribution.create(pd);
	}

}
