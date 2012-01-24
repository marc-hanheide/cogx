/**
 * 
 */
package de.dfki.lt.tr.beliefs.data.genericproxies;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.data.abstractproxies.ManagedContent;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribList;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author marc
 * 
 */
public class GenericIndependentDistributionList<T extends Distribution<?>>
		extends ManagedContent<CondIndependentDistribList, ProbDistribution, T>
		implements Collection<T> {

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
	public boolean add(T arg0) {
		return get().distribs.add(arg0.get());
	}

	@Override
	public boolean addAll(Collection<? extends T> arg0) {
		throw new RuntimeException("addAll not implemented yet");
		// TODO Auto-generated method stub
	}

	@Override
	public void clear() {
		get().distribs.clear();

	}

	@Override
	public boolean contains(Object arg0) {
		return get().distribs.contains(arg0);
	}

	@Override
	public boolean containsAll(Collection<?> arg0) {
		return get().distribs.containsAll(arg0);
	}

	@Override
	public boolean isEmpty() {
		return get().distribs.isEmpty();
	}

	@Override
	public Iterator<T> iterator() {
		Set<T> s = new HashSet<T>();
		for (ProbDistribution pd : get().distribs)
			s.add(_factory.create(pd));
		return s.iterator();
	}

	@Override
	public boolean remove(Object arg0) {
		return get().distribs.remove(arg0);
	}

	@Override
	public boolean removeAll(Collection<?> arg0) {
		throw new RuntimeException("removeAll not implemented yet");
		// TODO Auto-generated method stub
	}

	@Override
	public boolean retainAll(Collection<?> arg0) {
		throw new RuntimeException("retainAll not implemented yet");
		// TODO Auto-generated method stub
	}

	@Override
	public int size() {
		return get().distribs.size();
	}

	@Override
	public Object[] toArray() {
		throw new RuntimeException("retainAll not implemented yet");
		// TODO Auto-generated method stub
	}

	@Override
	public <T2> T2[] toArray(T2[] arg0) {
		throw new RuntimeException("retainAll not implemented yet");
		// TODO Auto-generated method stub
	}

}
