/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.abstractproxies;

/**
 * @author marc
 * 
 * @param <T>
 *            the type of proxy that is generate by this factory.
 */
public abstract class AbstractProxyFactory<C extends Ice.Object, T extends Proxy<? extends C>>
		implements ProxyFactory<C, T> {

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory#create(de.dfki
	 * .lt.tr.beliefs.data.abstractproxies.Proxy)
	 */
	@Override
	public T create(Proxy<? extends C> proxy) {
		return create(proxy.get());
	}

}
