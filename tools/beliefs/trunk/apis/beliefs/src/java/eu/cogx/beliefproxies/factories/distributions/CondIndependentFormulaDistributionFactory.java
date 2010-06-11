/**
 * 
 */
package eu.cogx.beliefproxies.factories.distributions;

import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy;

/**
 * @author marc
 * 
 */
public class CondIndependentFormulaDistributionFactory implements
		ProxyFactory<CondIndependentFormulaDistributionsProxy> {

	@Override
	public CondIndependentFormulaDistributionsProxy create(Ice.Object pd) {
		return new CondIndependentFormulaDistributionsProxy(pd);
	}

	@Override
	public CondIndependentFormulaDistributionsProxy create(Proxy<? extends Ice.Object> proxy) {
		return create(proxy.get());
	}

}
