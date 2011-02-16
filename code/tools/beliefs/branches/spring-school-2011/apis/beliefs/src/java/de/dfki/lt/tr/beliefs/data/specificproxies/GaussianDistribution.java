/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.specificproxies;

import de.dfki.lt.tr.beliefs.data.Gaussian;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.factories.specific.GaussianFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author marc
 * 
 */
public class GaussianDistribution extends
		GenericBasicDistribution<Gaussian> {

	public static GaussianDistribution create(ProbDistribution o) {
		return new GaussianDistribution(o);
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected GaussianDistribution(
			ProbDistribution content) {
		super(GaussianFactory.get(), content);
	}
}
