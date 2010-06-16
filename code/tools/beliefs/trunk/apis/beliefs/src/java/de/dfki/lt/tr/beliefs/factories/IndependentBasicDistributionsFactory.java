/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;


import de.dfki.lt.tr.beliefs.data.IndependentBasicDistributions;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentBasicDistributionsFactory implements
		ProxyFactory<IndependentBasicDistributions> {

	@Override
	public IndependentBasicDistributions create(Ice.Object pd) {
		return IndependentBasicDistributions.create(pd);
	}

	@Override
	public IndependentBasicDistributions create(Proxy<? extends Ice.Object> proxy) {
		return create(proxy.get());
	}

}
