/**
 * 
 */
package eu.cogx.beliefproxies.proxies.distributions;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import eu.cogx.beliefproxies.factories.distributions.BasicProbDistributionFactory;

/**
 * @author marc
 * 
 */
public class CondIndependentBasicDistributionsProxy extends
		CondIndependentDistributionProxy<BasicProbDistributionFactory> {

	public CondIndependentBasicDistributionsProxy(Object content) {
		super(new BasicProbDistributionFactory(), content);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy
	 * #get(java.lang.Object)
	 */
	@Override
	public BasicProbDistributionProxy<?> get(java.lang.Object arg0) {
		return factory.create(_content.distribs.get(arg0));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy
	 * #put(java.lang.String,
	 * eu.cogx.beliefproxies.proxies.distributions.DistributionProxy)
	 */
	@Override
	public BasicProbDistributionProxy<?> put(String arg0,
			DistributionProxy<? extends ProbDistribution> arg1) {
		return factory.create(_content.distribs.put(arg0, arg1.get()));

	}

}
