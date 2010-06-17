/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories.specific;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class FormulaDistributionFactory extends
		AbstractProxyFactory<ProbDistribution, FormulaDistribution> {

	private static FormulaDistributionFactory singleton = null;

	public static FormulaDistributionFactory get() {
		if (singleton == null)
			singleton = new FormulaDistributionFactory();
		return singleton;
	}

	@Override
	public FormulaDistribution create(ProbDistribution pd) {
		return FormulaDistribution.create(pd);
	}

}
