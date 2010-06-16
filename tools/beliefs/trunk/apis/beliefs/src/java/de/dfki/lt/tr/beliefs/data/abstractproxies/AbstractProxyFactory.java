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
public abstract class AbstractProxyFactory<T extends Proxy<?>> implements
		ProxyFactory<T> {

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory#create(de.dfki
	 * .lt.tr.beliefs.data.abstractproxies.Proxy)
	 */
	@Override
	public T create(Proxy<? extends Ice.Object> proxy) {
		return create(proxy.get());
	}

}
