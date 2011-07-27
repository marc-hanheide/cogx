/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.genericproxies;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import de.dfki.lt.tr.beliefs.data.abstractproxies.ManagedContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribList;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author marc
 * 
 */
public class GenericIndependentDistributionList<T extends Distribution<?>>
		extends ManagedContent<CondIndependentDistribList, ProbDistribution, T>
		implements Collection<T> {

	public void add(int index, T element) {
		get().distribs.add(index, element.get());
	}

	public boolean add(T e) {
		return get().distribs.add(e.get());
	}

	public void clear() {
		get().distribs.clear();
	}

	public boolean contains(Object o) {
		return get().distribs.contains(o);
	}

	public boolean containsAll(Collection<?> c) {
		return get().distribs.containsAll(c);
	}

	public boolean equals(Object o) {
		return get().distribs.equals(o);
	}

	public T get(int index) {
		ProbDistribution entry = get().distribs.get(index);
		if (entry == null)
			return null;
		return _factory.create(entry);
	}

	public int hashCode() {
		return get().distribs.hashCode();
	}

	public int indexOf(Object o) {
		return get().distribs.indexOf(o);
	}

	public boolean isEmpty() {
		return get().distribs.isEmpty();
	}

	public Iterator<T> iterator() {
		Set<T> set = new HashSet<T>();
		for (ProbDistribution pd : get().distribs) {
			set.add(_factory.create(pd));
		}
		return set.iterator();
	}

	public int lastIndexOf(Object o) {
		return get().distribs.lastIndexOf(o);
	}

	public T remove(int index) {
		return _factory.create(get().distribs.remove(index));
	}

	public boolean remove(Object o) {
		return get().distribs.remove(o);
	}

	public boolean removeAll(Collection<?> c) {
		return get().distribs.removeAll(c);
	}

	public boolean retainAll(Collection<?> c) {
		return get().distribs.retainAll(c);
	}

	// public T set(int index, T element) {
	// return get().distribs.set(index, element);
	// }

	public int size() {
		return get().distribs.size();
	}

	// public List<T> subList(int fromIndex, int toIndex) {
	// return get().distribs.subList(fromIndex, toIndex);
	// }

	public Object[] toArray() {
		return get().distribs.toArray();
	}

	public <T2> T2[] toArray(T2[] a) {
		return get().distribs.toArray(a);
	}

	public static <T2 extends Distribution<?>> GenericIndependentDistributionList<T2> create(
			ProxyFactory<ProbDistribution, ? extends T2> factory,
			ProbDistribution pd) {
		return new GenericIndependentDistributionList<T2>(factory, pd);
	}

	protected GenericIndependentDistributionList(
			ProxyFactory<ProbDistribution, ? extends T> factory,
			ProbDistribution content) {
		super(CondIndependentDistribList.class, factory, content);
	}

	@Override
	public boolean addAll(Collection<? extends T> c) {
		for (T e : c) {
			_content.distribs.add(e.get());
		}
		return false;
	}

}
