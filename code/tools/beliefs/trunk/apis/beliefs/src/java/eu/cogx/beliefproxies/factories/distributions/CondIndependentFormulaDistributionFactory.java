/**
 * 
 */
package eu.cogx.beliefproxies.factories.distributions;

import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import Ice.Object;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.distributions.BasicProbDistributionProxy;
import eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy;

/**
 * @author marc
 * 
 */
public class CondIndependentFormulaDistributionFactory implements
		ProxyFactory<CondIndependentFormulaDistributionsProxy> {

	@Override
	public CondIndependentFormulaDistributionsProxy create(Object pd) {
		return new CondIndependentFormulaDistributionsProxy(pd);
	}

	@Override
	public CondIndependentFormulaDistributionsProxy create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
