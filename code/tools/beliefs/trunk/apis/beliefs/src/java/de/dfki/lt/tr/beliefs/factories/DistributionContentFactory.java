/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.genericproxies.DistributionContent;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class DistributionContentFactory extends
		AbstractProxyFactory<DistributionContent<DistributionValues>> {

	@Override
	public DistributionContent<DistributionValues> create(Object pd) {
		return new DistributionContent<DistributionValues>(
				DistributionValues.class, pd);
	}

}
