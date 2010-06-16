/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.DistributionContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class DistributionContentFactory extends
		AbstractProxyFactory<DistributionContent<?, ?>> {

	@Override
	public DistributionContent<?, ?> create(Object pd) {
		return DistributionContent.create(new GenericFactory(), pd);
	}


}
