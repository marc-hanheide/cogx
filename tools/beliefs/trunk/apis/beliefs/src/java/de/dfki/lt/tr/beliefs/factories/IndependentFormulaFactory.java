/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentFormulaFactory extends
		AbstractProxyFactory<ProbDistribution, IndependentDistribution> {

	@Override
	public IndependentDistribution create(ProbDistribution pd) {
		return IndependentDistribution.create(pd);
	}
}
