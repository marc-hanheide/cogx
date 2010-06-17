/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.DistributionContent;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.factories.BasicDistributionFactory;
import de.dfki.lt.tr.beliefs.factories.DistributionContentFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author marc
 * 
 */
public class BasicDistribution extends
		GenericBasicDistribution<DistributionContent<DistributionValues>> {

	public static BasicDistribution create(ProbDistribution o) {
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
	protected BasicDistribution(ProbDistribution content) {
		super(DistributionContentFactory.get(), content);
	}

	public static BasicDistribution create(
			Distribution<ProbDistribution> distribution) {
		return create(BasicDistributionFactory.get(), distribution);
	}

}
