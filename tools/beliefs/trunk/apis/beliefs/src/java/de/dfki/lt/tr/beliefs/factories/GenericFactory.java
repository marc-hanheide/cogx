/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class GenericFactory extends
		AbstractProxyFactory<Proxy<? extends Ice.Object>> {

	@Override
	public Proxy<Ice.Object> create(Ice.Object pd) {
		return new Proxy<Ice.Object>(Ice.Object.class, pd) {
		};
	}

}
