/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.abstractproxies.ManagedContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;

/**
 * @author marc
 * 
 */
public class BasicDistribution<T extends ProxyFactory<? extends Proxy<? extends DistributionValues>>>
		extends ManagedContent<BasicProbDistribution, T> {
	/**
	 * @param class1
	 * @param content
	 */
	public BasicDistribution(T factory, Ice.Object content) {
		super(BasicProbDistribution.class, factory, content);
	}

	public Proxy<? extends DistributionValues> getDistribution() {
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
