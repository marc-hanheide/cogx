/**
 * 
 */
package eu.cogx.beliefproxies.proxies.distributions;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import eu.cogx.beliefproxies.factories.ProxyFactory;

/**
 * @author marc
 * 
 */
public class CondIndependentDistributionProxy<T extends ProxyFactory<? extends DistributionProxy<?>>>
		extends DistributionProxy<CondIndependentDistribs> implements
		Map<String, DistributionProxy<?>> {

	protected final T factory;

	public CondIndependentDistributionProxy(T factory, Ice.Object content) {
		super(CondIndependentDistribs.class, content);
		this.factory = factory;
	}

	/**
	 * 
	 * @see java.util.Map#clear()
	 */
	public void clear() {
		_content.distribs.clear();
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#containsKey(java.lang.Object)
	 */
	public boolean containsKey(Object arg0) {
		return _content.distribs.containsKey(arg0);
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#containsValue(java.lang.Object)
	 */
	public boolean containsValue(Object arg0) {
		return _content.distribs.containsValue(arg0);
	}

	/**
	 * @return
	 * @see java.util.Map#entrySet()
	 * 
	 */
	public Set<Entry<String, DistributionProxy<? extends ProbDistribution>>> entrySet() {
		Map<String, DistributionProxy<? extends ProbDistribution>> result = new HashMap<String, DistributionProxy<? extends ProbDistribution>>();
		for (Entry<String, ProbDistribution> pd : _content.distribs.entrySet()) {
			result.put(pd.getKey(), factory.create(pd.getValue()));
		}
		return result.entrySet();
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#get(java.lang.Object)
	 */
	public DistributionProxy<? extends ProbDistribution> get(Object arg0) {
		ProbDistribution entry = _content.distribs.get(arg0);
		if (entry == null)
			return null;
		return factory.create(entry);
	}

	/**
	 * @return
	 * @see java.util.Map#hashCode()
	 */
	public int hashCode() {
		return _content.distribs.hashCode();
	}

	/**
	 * @return
	 * @see java.util.Map#isEmpty()
	 */
	public boolean isEmpty() {
		return _content.distribs.isEmpty();
	}

	/**
	 * @return
	 * @see java.util.Map#keySet()
	 */
	public Set<String> keySet() {
		return _content.distribs.keySet();
	}

	/**
	 * @param arg0
	 * @param arg1
	 * @return
	 * @see java.util.Map#put(java.lang.Object, java.lang.Object)
	 */
	public DistributionProxy<? extends ProbDistribution> put(String arg0,
			DistributionProxy<? extends ProbDistribution> arg1) {
		return factory.create(_content.distribs.put(arg0, arg1.get()));
	}

	/**
	 * @param arg0
	 * @see java.util.Map#putAll(java.util.Map)
	 */
	public void putAll(
			Map<? extends String, ? extends DistributionProxy<? extends ProbDistribution>> arg0) {
		for (Entry<? extends String, ? extends DistributionProxy<? extends ProbDistribution>> e : arg0
				.entrySet()) {
			_content.distribs.put(e.getKey(), e.getValue().get());
		}
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#remove(java.lang.Object)
	 */
	public DistributionProxy<? extends ProbDistribution> remove(Object arg0) {
		return factory.create(_content.distribs.remove(arg0));
	}

	/**
	 * @return
	 * @see java.util.Map#size()
	 */
	public int size() {
		return _content.distribs.size();
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		String result= getClass().getSimpleName() + " [";
		for (Entry<String, ProbDistribution> e : _content.distribs.entrySet()) {
			result+=e.getKey() + "=>" + factory.create(e.getValue()).toString()+" ";
		}
		result+="] ";
		return result;
	}
	/**
	 * @return
	 * @see java.util.Map#values()
	 */
	public Collection<DistributionProxy<? extends ProbDistribution>> values() {
		Vector<DistributionProxy<? extends ProbDistribution>> result = new Vector<DistributionProxy<? extends ProbDistribution>>(
				_content.distribs.size());
		for (ProbDistribution p : _content.distribs.values()) {
			result.add(factory.create(p));
		}
		return result;

	}


}
