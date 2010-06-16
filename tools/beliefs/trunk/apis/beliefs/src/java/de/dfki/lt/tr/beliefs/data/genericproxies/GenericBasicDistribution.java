/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.genericproxies;

import de.dfki.lt.tr.beliefs.data.abstractproxies.ManagedContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;

/**
 * @author marc
 * 
 */
public class GenericBasicDistribution<T extends DistributionContent<?>> extends
		ManagedContent<BasicProbDistribution, T> {

	public static <T2 extends DistributionContent<?>> GenericBasicDistribution<T2> create(
			ProxyFactory<? extends T2> factory, Ice.Object o) {
		return new GenericBasicDistribution<T2>(factory, o);
	}

	/**
	 * @param class1
	 * @param content
	 */
	protected GenericBasicDistribution(ProxyFactory<? extends T> factory,
			Ice.Object content) {
		super(BasicProbDistribution.class, factory, content);
	}

	public T getDistribution() {
		return _factory.create(_content.values);
	}

	public void setDistribution(T d) {
		_content.values=d.get();
	}

	public String getId() {
		return _content.key;
	}

	public void setId(String key) {
		_content.key = key;
	}

	@Override
	public String toString() {
		String result = getClass().getSimpleName() + " [" + _content.key + "=";
		result += _factory.create(_content.values).toString();
		result += "] ";
		return result;
	}
}
