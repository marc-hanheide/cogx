/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.abstractproxies;



/**
 * @author marc
 *
 */
public interface ProxyFactory<C extends Ice.Object, T extends Proxy<? extends C>> {
	public T create(C o);
	public T create(Proxy<? extends C> proxy);
}
