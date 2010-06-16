/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author marc
 * 
 */
public class DistributionContent<T extends DistributionValues, P extends Proxy<? extends Ice.Object>>
		extends Proxy<T> {
	public static <P2 extends Proxy<? extends Ice.Object>> DistributionContent<DistributionValues,P2> create(
			ProxyFactory<? extends P2> genericFactory, Ice.Object pd) {
		return new DistributionContent<DistributionValues, P2>(
				DistributionValues.class, genericFactory, pd);
	}

	protected final ProxyFactory<? extends P> _factory;

	public DistributionContent(Class<? extends T> type,
			ProxyFactory<? extends P> factory, Ice.Object content) {
		super(type, content);
		this._factory = factory;
	}

}
