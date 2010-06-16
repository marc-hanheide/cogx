/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.BasicDistribution;
import de.dfki.lt.tr.beliefs.data.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class FormulaDistributionFactory implements
		ProxyFactory<FormulaDistribution> {

	@Override
	public FormulaDistribution create(Object pd) {
		return FormulaDistribution.create(pd);
	}

	@Override
	public FormulaDistribution create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
