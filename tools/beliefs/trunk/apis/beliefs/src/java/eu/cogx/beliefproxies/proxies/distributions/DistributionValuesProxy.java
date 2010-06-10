/**
 * 
 */
package eu.cogx.beliefproxies.proxies.distributions;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import eu.cogx.beliefproxies.proxies.Proxy;

/**
 * @author marc
 *
 */
public class DistributionValuesProxy extends Proxy<DistributionValues> {

	public DistributionValuesProxy(Object content) {
		super(DistributionValues.class, content);
	}

}
