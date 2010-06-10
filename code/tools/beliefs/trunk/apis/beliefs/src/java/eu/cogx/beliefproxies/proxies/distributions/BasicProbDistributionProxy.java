/**
 * 
 */
package eu.cogx.beliefproxies.proxies.distributions;

import java.util.Map.Entry;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;

/**
 * @author marc
 * 
 */
public class BasicProbDistributionProxy<T extends ProxyFactory<? extends Proxy<? extends DistributionValues>>>
		extends DistributionProxy<BasicProbDistribution> {
	protected final T factory;

	/**
	 * @param class1
	 * @param content
	 */
	public BasicProbDistributionProxy(T factory, Object content) {
		super(BasicProbDistribution.class, content);
		this.factory = factory;
	}

	public Proxy<? extends DistributionValues> getDistribution() {
		return factory.create(_content.values);
	}

	public String getId() {
		return _content.key;
	}

	@Override
	public String toString() {
		String result = getClass().getSimpleName() + " [" + _content.key + "=";
		result+=factory.create(_content.values).toString();
		result += "] ";
		return result;
	}
}
