/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;


import de.dfki.lt.tr.beliefs.data.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;


/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaDistributionsFactory implements
		ProxyFactory<IndependentFormulaDistributions> {

	@Override
	public IndependentFormulaDistributions create(Ice.Object pd) {
		return IndependentFormulaDistributions.create(pd);
	}

	@Override
	public IndependentFormulaDistributions create(Proxy<? extends Ice.Object> proxy) {
		return create(proxy.get());
	}

}
