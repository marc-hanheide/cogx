/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.genericproxies;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.data.abstractproxies.ManagedContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author marc
 * 
 */
public class GenericIndependentDistribution<T extends Distribution<?>> extends
		ManagedContent<CondIndependentDistribs, ProbDistribution, T> implements
		Map<String, T> {

	public static <T2 extends Distribution<?>> GenericIndependentDistribution<T2> create(
			ProxyFactory<ProbDistribution, ? extends T2> factory,
			ProbDistribution pd) {
		return new GenericIndependentDistribution<T2>(factory, pd);
	}

	protected GenericIndependentDistribution(
			ProxyFactory<ProbDistribution, ? extends T> factory,
			ProbDistribution content) {
		super(CondIndependentDistribs.class, factory, content);
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
	public Set<Entry<String, T>> entrySet() {
		Map<String, T> result = new HashMap<String, T>();
		for (Entry<String, ProbDistribution> pd : _content.distribs.entrySet()) {
			result.put(pd.getKey(), _factory.create(pd.getValue()));
		}
		return result.entrySet();
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#get(java.lang.Object)
	 */
	public T get(Object arg0) {
		ProbDistribution entry = _content.distribs.get(arg0);
		if (entry == null)
			return null;
		return _factory.create(entry);
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#get(java.lang.Object)
	 */
	public <T2 extends Proxy<? extends ProbDistribution>> T2 get(Object arg0,
			ProxyFactory<ProbDistribution, T2> factory) {
		ProbDistribution entry = _content.distribs.get(arg0);
		if (entry == null)
			return null;
		return factory.create(entry);
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

	// /**
	// * @param arg0
	// * @param arg1
	// * @return
	// * @see java.util.Map#put(java.lang.Object, java.lang.Object)
	// */
	// public T put(String arg0,
	// T arg1) {
	// return _factory.create(_content.distribs.put(arg0, arg1.get()));
	// }

	/**
	 * @param arg0
	 * @param arg1
	 * @return
	 * @see java.util.Map#put(java.lang.Object, java.lang.Object)
	 */
	public T put(String arg0, T arg1) {
		if ((arg1.get() instanceof BasicProbDistribution)) {
			// write the new key
			((BasicProbDistribution) arg1.get()).key = arg0;
		}
		ProbDistribution c = _content.distribs.put(arg0, arg1.get());
		if (c != null)
			return _factory.create(c);
		else
			return null;
	}

	/**
	 * @param arg0
	 * @see java.util.Map#putAll(java.util.Map)
	 */
	public void putAll(Map<? extends String, ? extends T> arg0) {
		for (Entry<? extends String, ? extends Distribution<? extends ProbDistribution>> e : arg0
				.entrySet()) {
			_content.distribs.put(e.getKey(), e.getValue().get());
		}
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#remove(java.lang.Object)
	 */
	public T remove(Object arg0) {
		ProbDistribution removed = _content.distribs.remove(arg0);
		if (removed != null) {
			return _factory.create(removed);
		} else {
			return null;
		}
	}

	/**
	 * @return
	 * @see java.util.Map#size()
	 */
	public int size() {
		return _content.distribs.size();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		String result = getClass().getSimpleName() + " [";
		for (Entry<String, ProbDistribution> e : _content.distribs.entrySet()) {
			result += e.getKey() + "=>"
					+ _factory.create(e.getValue()).toString() + " ";
		}
		result += "] ";
		return result;
	}

	/**
	 * @return
	 * @see java.util.Map#values()
	 */
	public Collection<T> values() {
		Vector<T> result = new Vector<T>(_content.distribs.size());
		for (ProbDistribution p : _content.distribs.values()) {
			result.add(_factory.create(p));
		}
		return result;

	}

}
