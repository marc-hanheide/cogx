/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories.specific;

import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentDistribution> {

	private static IndependentFormulaFactory singleton = null;

	public static IndependentFormulaFactory get() {
		if (singleton == null)
			singleton = new IndependentFormulaFactory();
		return singleton;
	}

	@Override
	public IndependentDistribution create(ProbDistribution pd) {
		return IndependentDistribution.create(pd);
	}
}
