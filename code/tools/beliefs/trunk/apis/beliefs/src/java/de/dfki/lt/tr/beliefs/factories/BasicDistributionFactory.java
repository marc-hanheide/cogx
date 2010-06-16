/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.BasicDistribution;
import de.dfki.lt.tr.beliefs.data.DistributionContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class BasicDistributionFactory
		implements
		DefaultProxyFactory<BasicDistribution<? extends DistributionContent<?, ?>>> {

	@Override
	public BasicDistribution<? extends DistributionContent<?, ?>> create() {
		return create(new BasicProbDistribution("", new DistributionValues()));
	}

	@Override
	public BasicDistribution<? extends DistributionContent<?, ?>> create(
			Object pd) {
		return BasicDistribution
				.create(new DistributionContentFactory(), pd);
	}

	@Override
	public BasicDistribution<? extends DistributionContent<?, ?>> create(
			Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

	public BasicDistribution<? extends DistributionContent<?, ?>> create(
			String key, Proxy<? extends DistributionValues> values) {
		return create(new BasicProbDistribution(key, values.get()));
	}
}
