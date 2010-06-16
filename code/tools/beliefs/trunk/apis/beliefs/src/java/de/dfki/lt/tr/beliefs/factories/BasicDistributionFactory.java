/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.BasicDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class BasicDistributionFactory implements
		ProxyFactory<BasicDistribution> {

	@Override
	public BasicDistribution create(Object pd) {
		return BasicDistribution.create(pd);
	}

	@Override
	public BasicDistribution create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
