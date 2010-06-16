/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistribution;
import de.dfki.lt.tr.beliefs.factories.FormulaDistributionFactory;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaDistributions extends
		GenericIndependentDistribution<FormulaDistribution> {

	public static IndependentFormulaDistributions create(Ice.Object o) {
		return new IndependentFormulaDistributions(o);
	}

	protected IndependentFormulaDistributions(Object content) {
		super(new FormulaDistributionFactory(), content);
	}

}
