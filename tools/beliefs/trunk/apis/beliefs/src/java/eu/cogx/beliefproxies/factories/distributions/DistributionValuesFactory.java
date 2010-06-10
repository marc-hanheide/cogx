/**
 * 
 */
package eu.cogx.beliefproxies.factories.distributions;

import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.distributions.DistributionValuesProxy;

/**
 * @author marc
 *
 */
public class DistributionValuesFactory implements ProxyFactory<DistributionValuesProxy> {

	@Override
	public DistributionValuesProxy create(Ice.Object pd) {
		return new DistributionValuesProxy(pd);
	}

	@Override
	public DistributionValuesProxy create(Proxy<? extends Ice.Object> proxy) {
		return create(proxy.get());
	}

}
