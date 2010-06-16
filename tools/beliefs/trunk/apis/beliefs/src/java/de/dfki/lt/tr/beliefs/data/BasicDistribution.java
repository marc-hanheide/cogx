/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.abstractproxies.ManagedContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;

/**
 * @author marc
 * 
 */
public class BasicDistribution<T extends DistributionContent<?, ?>> extends
		ManagedContent<BasicProbDistribution, T> {

	public static <T2 extends DistributionContent<?, ?>> BasicDistribution<T2> create(
			ProxyFactory<? extends T2> factory, Ice.Object o) {
		return new BasicDistribution<T2>(factory, o);
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected BasicDistribution(ProxyFactory<? extends T> factory,
			Ice.Object content) {
		super(BasicProbDistribution.class, factory, content);
	}

	public T getDistribution() {
		return _factory.create(_content.values);
	}

	public String getId() {
		return _content.key;
	}

	@Override
	public String toString() {
		String result = getClass().getSimpleName() + " [" + _content.key + "=";
		result += _factory.create(_content.values).toString();
		result += "] ";
		return result;
	}
}
