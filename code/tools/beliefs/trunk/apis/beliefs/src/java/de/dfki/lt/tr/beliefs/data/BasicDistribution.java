/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.genericproxies.DistributionContent;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.factories.DistributionContentFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author marc
 * 
 */
public class BasicDistribution extends
		GenericBasicDistribution<DistributionContent<DistributionValues>> {

	public static BasicDistribution create(Ice.Object o) {
		return new BasicDistribution(o);
	}

	public static BasicDistribution create() {
		BasicProbDistribution o = new BasicProbDistribution("",
				new DistributionValues());
		return new BasicDistribution(o);
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected BasicDistribution(Ice.Object content) {
		super(new DistributionContentFactory(), content);
	}
}
