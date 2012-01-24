/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import de.dfki.lt.tr.beliefs.data.BasicDistribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistributionList;
import de.dfki.lt.tr.beliefs.factories.BasicDistributionFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentBasicDistributionList extends
		GenericIndependentDistributionList<BasicDistribution> {

	public static IndependentBasicDistributionList create(ProbDistribution o) {
		return new IndependentBasicDistributionList(o);
	}

	protected IndependentBasicDistributionList(ProbDistribution content) {
		super(BasicDistributionFactory.get(), content);
	}

}
