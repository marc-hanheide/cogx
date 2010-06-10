/**
 * 
 */
package eu.cogx.beliefproxies.factories.distributions;

import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import Ice.Object;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.distributions.BasicProbDistributionProxy;

/**
 * @author marc
 * 
 */
public class BasicProbDistributionFactory implements
		ProxyFactory<BasicProbDistributionProxy<?>> {

	@Override
	public BasicProbDistributionProxy<?> create(Object pd) {
		return new BasicProbDistributionProxy<ProxyFactory<? extends Proxy<? extends DistributionValues>>>(
				new DistributionValuesFactory(), pd);
	}

	@Override
	public BasicProbDistributionProxy<?> create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
