/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.abstractproxies.ManagedContent;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.factories.GaussianFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;

/**
 * @author marc
 * 
 */
public class GaussianDistribution extends
		GenericBasicDistribution<Gaussian> {

	public static GaussianDistribution create(Ice.Object o) {
		return new GaussianDistribution(o);
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected GaussianDistribution(
			Ice.Object content) {
		super(new GaussianFactory(), content);
	}
}
