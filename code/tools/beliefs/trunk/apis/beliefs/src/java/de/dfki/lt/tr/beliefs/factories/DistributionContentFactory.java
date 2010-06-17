/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.genericproxies.DistributionContent;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class DistributionContentFactory
		extends
		AbstractProxyFactory<DistributionValues, DistributionContent<DistributionValues>> {

	@Override
	public DistributionContent<DistributionValues> create(DistributionValues pd) {
		return new DistributionContent<DistributionValues>(
				DistributionValues.class, pd);
	}

}
