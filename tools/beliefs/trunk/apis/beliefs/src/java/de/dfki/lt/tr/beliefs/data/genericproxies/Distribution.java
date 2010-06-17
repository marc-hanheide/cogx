package de.dfki.lt.tr.beliefs.data.genericproxies;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.factories.DistributionFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * a specialisation of a Proxy for probability distributions
 * 
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 * @param <T>
 *            the encapsulated type
 */
public class Distribution<T extends ProbDistribution> extends Proxy<T> {

	public static <T2 extends ProbDistribution> Distribution<T2> create(
			Class<? extends T2> type, ProbDistribution pd) {
		return new Distribution<T2>(type, pd);
	}

	protected Distribution(Class<? extends T> class1, ProbDistribution content) {
		super(class1, content);
	}
	
	public Distribution<ProbDistribution> asDistribution() {
		return DistributionFactory.get().create(this);
	}
	


}
