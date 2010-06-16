/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaFactory  implements ProxyFactory<IndependentDistribution> {

	@Override
	public IndependentDistribution create(Ice.Object pd) {
		return IndependentDistribution.create(pd);
	}

	@Override
	public IndependentDistribution create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
