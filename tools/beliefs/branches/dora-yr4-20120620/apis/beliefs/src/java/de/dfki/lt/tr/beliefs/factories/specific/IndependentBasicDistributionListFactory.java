/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories.specific;

import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentBasicDistributionList;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentBasicDistributionListFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentBasicDistributionList> {

	private static IndependentBasicDistributionListFactory singleton = null;

	public static IndependentBasicDistributionListFactory get() {
		if (singleton == null)
			singleton = new IndependentBasicDistributionListFactory();
		return singleton;
	}

	@Override
	public IndependentBasicDistributionList create(ProbDistribution pd) {
		return IndependentBasicDistributionList.create(pd);
	}

}
