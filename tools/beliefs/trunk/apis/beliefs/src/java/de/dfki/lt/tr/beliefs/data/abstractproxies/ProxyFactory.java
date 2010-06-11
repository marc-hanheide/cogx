/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.abstractproxies;



/**
 * @author marc
 *
 */
public interface ProxyFactory<T extends Proxy<? extends Ice.Object>> {
	public T create(Ice.Object pd);
	public T create(Proxy<? extends Ice.Object> proxy);
}
