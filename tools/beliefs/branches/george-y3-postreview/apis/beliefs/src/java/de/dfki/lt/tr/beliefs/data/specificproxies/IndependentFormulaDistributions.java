/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistribution;
import de.dfki.lt.tr.beliefs.factories.specific.FormulaDistributionFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaDistributions extends
		GenericIndependentDistribution<FormulaDistribution> {

	public static IndependentFormulaDistributions create(ProbDistribution o) {
		return new IndependentFormulaDistributions(o);
	}

	protected IndependentFormulaDistributions(ProbDistribution content) {
		super(FormulaDistributionFactory.get(), content);
	}



}
