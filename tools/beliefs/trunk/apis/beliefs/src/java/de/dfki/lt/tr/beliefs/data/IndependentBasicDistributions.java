/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistribution;
import de.dfki.lt.tr.beliefs.factories.BasicDistributionFactory;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentBasicDistributions extends
		GenericIndependentDistribution<BasicDistribution> {

	public static IndependentBasicDistributions create(Ice.Object o) {
		return new IndependentBasicDistributions(o);
	}

	protected IndependentBasicDistributions(Object content) {
		super(new BasicDistributionFactory(), content);
	}

}
