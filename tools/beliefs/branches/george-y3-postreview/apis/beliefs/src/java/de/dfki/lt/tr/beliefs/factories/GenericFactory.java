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
		AbstractProxyFactory<Ice.Object, Proxy<? extends Ice.Object>> {

	private static GenericFactory singleton = null;

	public static GenericFactory get() {
		if (singleton == null)
			singleton = new GenericFactory();
		return singleton;
	}

	@Override
	public Proxy<Ice.Object> create(Ice.Object pd) {
		return new Proxy<Ice.Object>(Ice.Object.class, pd) {
		};
	}

}
